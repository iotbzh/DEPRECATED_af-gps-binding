/*
 * Copyright (C) 2016 "IoT.bzh"
 * Author José Bollo <jose.bollo@iot.bzh>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <netdb.h>
#include <fcntl.h>
#include <math.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <json-c/json.h>

#include <systemd/sd-event.h>

#include <afb/afb-binding.h>

#define NAUTICAL_MILE_IN_METER                     1852
#define MILE_IN_METER                              1609.344
#define KNOT_TO_METER_PER_SECOND                   0.5144444444         /* 1852 / 3600 */
#define METER_PER_SECOND_TO_KNOT                   1.943844492          /* 3600 / 1852 */
#define METER_PER_SECOND_TO_KILOMETER_PER_HOUR     3.6                  /* 3600 / 1000 */
#define METER_PER_SECOND_TO_MILE_PER_HOUR          2.236936292          /* 3600 / 1609.344 */

/*
 * references:
 *
 *       https://www.w3.org/TR/geolocation-API/
 *       http://www.gpsinformation.org/dale/nmea.htm
 */

struct flags {
	unsigned time: 1;
	unsigned latitude: 1;
	unsigned longitude: 1;
	unsigned altitude: 1;
	unsigned speed: 1;
	unsigned track: 1;
};

struct gps {
	struct flags set;

	uint32_t time;
	double latitude;
	double longitude;
	double altitude;
	double speed;
	double track;
};

enum type {
	type_wgs84,
	type_dms_kmh,
	type_dms_mph,
	type_dms_kn,
	type_COUNT,
	type_DEFAULT = type_wgs84,
	type_INVALID = -1
};

struct event;
struct period;

struct period {
	struct period *next;
	struct event *events;
	uint32_t period;
	uint32_t last;
};

struct event {
	struct event *next;
	struct event *byid;
	struct period *period;
	const char *name;
	struct afb_event event;
	enum type type;
	int id;
};

static const char * const type_NAMES[type_COUNT] = {
	"WGS84",
	"DMS.km/h",
	"DMS.mph",
	"DMS.kn"
};

/*
 * the interface to afb-daemon
 */
const struct afb_binding_interface *afbitf;

static struct gps frames[10];
static int frameidx;
static int newframes;

static struct json_object *time_ms;
static struct json_object *latitude_wgs;
static struct json_object *longitude_wgs;
static struct json_object *latitude_dms;
static struct json_object *longitude_dms;
static struct json_object *altitude_m;
static struct json_object *speed_ms;
static struct json_object *speed_kmh;
static struct json_object *speed_mph;
static struct json_object *speed_kn;
static struct json_object *track_d;

static struct json_object *positions[type_COUNT];

static struct period *periods;
static struct event *events;

/* declare the connection routine */
static int nmea_connect();

/*
 * Creates the JSON representation for Degree Minute Second representation of coordinates
 */
static struct json_object *new_dms(double a, int islat)
{
	char buffer[50], pos;
	double D, M;

	if (islat) {
		if (a >= 0)
			pos = 'N';
		else {
			a = -a;
			pos = 'S';
		}
	} else {
		if (a <= 180)
			pos = 'E';
		else {
			a = 360 - a;
			pos = 'W';
		}
	}
	D = floor(a);
	a = (a - D) * 60;
	M = floor(a);
	a = (a - M) * 60;
	sprintf(buffer, "%d°%d'%.3f\"%c", (int)D, (int)M, a, pos);
	return json_object_new_string(buffer);
}

/*
 * adds the value (with reference count increment) if not null
 */
static void addif(struct json_object *obj, const char *name, struct json_object *val)
{
	if (val != NULL)
		json_object_object_add(obj, name, json_object_get(val));
}

/*
 * release the object (put) and reset the pointer to null
 */
static void clear(struct json_object **obj)
{
	json_object_put(*obj);
	*obj = NULL;
}

/*
 * get the last/current position of type
 */
static struct json_object *position(enum type type)
{
	struct json_object *result;
	struct gps *g0;

	/* clean on new frame */
	if (newframes) {
		clear(&time_ms);
		clear(&latitude_wgs);
		clear(&longitude_wgs);
		clear(&latitude_dms);
		clear(&longitude_dms);
		clear(&altitude_m);
		clear(&speed_ms);
		clear(&speed_kmh);
		clear(&speed_mph);
		clear(&speed_kn);
		clear(&track_d);
		clear(&positions[type_wgs84]);
		clear(&positions[type_dms_kmh]);
		clear(&positions[type_dms_mph]);
		clear(&positions[type_dms_kn]);
		newframes = 0;
	}

	/* get the result */
	result = positions[type];
	if (result == NULL) {
		DEBUG(afbitf, "building position for type %s", type_NAMES[type]);

		/* should build the result */
		g0 = &frames[frameidx];
		result = json_object_new_object();
		if (result == NULL)
			return NULL;
		positions[type] = result;

		/* set the result type */
		json_object_object_add(result, "type", json_object_new_string(type_NAMES[type]));

		/* build time, altitude and track */
		if (time_ms == NULL && g0->set.time)
			time_ms = json_object_new_double (g0->time);
		addif(result, "time", time_ms);
		if (altitude_m == NULL && g0->set.altitude)
			altitude_m = json_object_new_double (g0->altitude);
		addif(result, "altitude", altitude_m);
		if (track_d == NULL && g0->set.track)
			track_d = json_object_new_double (g0->track);
		addif(result, "track", track_d);

		/* build position */
		switch (type) {
		default:
		case type_wgs84:
			if (latitude_wgs == NULL && g0->set.latitude)
				latitude_wgs = json_object_new_double (g0->latitude);
			addif(result, "latitude", latitude_wgs);
			if (longitude_wgs == NULL && g0->set.longitude)
				longitude_wgs = json_object_new_double (g0->longitude);
			addif(result, "longitude", longitude_wgs);
			break;
		case type_dms_kmh:
		case type_dms_mph:
		case type_dms_kn:
			if (latitude_dms == NULL && g0->set.latitude)
				latitude_dms = new_dms (g0->latitude, 1);
			addif(result, "latitude", latitude_dms);
			if (longitude_dms == NULL && g0->set.longitude)
				longitude_dms = new_dms (g0->longitude, 0);
			addif(result, "longitude", longitude_dms);
			break;
		}

		/* build speed */
		switch (type) {
		default:
		case type_wgs84:
			if (speed_ms == NULL && g0->set.speed)
				speed_ms = json_object_new_double (g0->speed);
			addif(result, "speed", speed_ms);
			break;
		case type_dms_kmh:
			if (speed_kmh == NULL && g0->set.speed)
				speed_kmh = json_object_new_double (g0->speed * METER_PER_SECOND_TO_KILOMETER_PER_HOUR);
			addif(result, "speed", speed_kmh);
			break;
		case type_dms_mph:
			if (speed_mph == NULL && g0->set.speed)
				speed_mph = json_object_new_double (g0->speed * METER_PER_SECOND_TO_MILE_PER_HOUR);
			addif(result, "speed", speed_mph);
			break;
		case type_dms_kn:
			if (speed_kn == NULL && g0->set.speed)
				speed_kn = json_object_new_double (g0->speed * METER_PER_SECOND_TO_KNOT);
			addif(result, "speed", speed_kn);
			break;
		}
	}

	return json_object_get(result);
}

/*
 * get the event handler of given id
 */
static struct event *event_of_id(int id)
{
	struct event *r = events;
	while(r && r->id != id)
		r = r->byid;
	return r;
}

/*
 * get the event handler for the type and the period
 */
static struct event *event_get(enum type type, int period)
{
	static int id;
	int mask;
	uint32_t perio;
	struct period *p, **pp, *np;
	struct event *e;

	/* normalize the period */
	period = period <= 100 ? 1 : period > 60000 ? 600 : (period / 100);
	mask = 31;
	while(period > (period & mask))
		mask <<= 1;
	perio = (uint32_t)(100 * (period & mask));

	/* search for the period */
	pp = &periods;
	p = *pp;
	while(p != NULL && p->period < perio) {
		pp = &p->next;
		p = *pp;
	}

	/* create the period if it misses */
	if (p == NULL || p->period != perio) {
		np = calloc(1, sizeof *p);
		if (np == NULL)
			return NULL;
		np->next = p;
		np->period = perio;
		*pp = np;
		p = np;
	}

	/* search the type */
	e = p->events;
	while(e != NULL && e->type != type)
		e = e->next;

	/* creates the type if needed */
	if (e == NULL) {
		e = calloc(1, sizeof *e);
		if (e == NULL)
			return NULL;

		e->name = "GPS"; /* TODO */
		e->event = afb_daemon_make_event(afbitf->daemon, e->name);
		if (e->event.itf == NULL) {
			free(e);
			return NULL;
		}

		e->next = p->events;
		e->byid = events;
		e->period = p;
		e->type = type;
		do {
			id++;
			if (id < 0)
				id = 1;
		} while(event_of_id(id) != NULL);
		e->id = id;
		p->events = e;
		events = e;
	}

	return e;
}

static void event_send()
{
	struct period *p, **pp;
	struct event *e, **pe, **peid;
	struct timeval tv;
	uint32_t now;

	/* skip if nothing is new */
	if (!newframes)
		return;

	/* computes now */
	gettimeofday(&tv, NULL);
	now = (uint32_t)(tv.tv_sec * 1000) + (uint32_t)(tv.tv_usec / 1000);

	/* iterates over the periods */
	pp = &periods;
	p = *pp;
	while (p != NULL) {
		if (p->events == NULL) {
			/* no event for the period, frees it */
			*pp = p->next;
			free(p);
		} else {
			if (p->period <= now - p->last) {
				/* its time to refresh */
				p->last = now;
				pe = &p->events;
				e = *pe;
				while (e != NULL) {
					/* sends the event */
					if (afb_event_push(e->event, position(e->type)) != 0)
						pe = &e->next;
					else {
						/* no more listeners, free the event */
						*pe = e->next;
						peid = &events;
						while (*peid != e)
							peid = &(*peid)->byid;
						*peid = e->byid;
						afb_event_drop(e->event);
						free(e);
					}
					e = *pe;
				}
			}
			pp = &p->next;
		}
		p = *pp;
	}
}

static int nmea_time(const char *text, uint32_t *result)
{
	uint32_t x;

	if (text[0] < '0' || text[0] > '2'
	 || text[1] < '0' || text[1] > (text[0] == '2' ? '3' : '9')
	 || text[2] < '0' || text[2] > '5'
	 || text[3] < '0' || text[3] > '9'
	 || text[4] < '0' || text[4] > '5'
	 || text[5] < '0' || text[5] > '9'
	 || (text[6] != 0 && text[6] != '.'))
		return 0;

	x = (uint32_t)(text[0] - '0');
	x = x * 10 + (uint32_t)(text[1]-'0');
	x = x *  6 + (uint32_t)(text[2]-'0');
	x = x * 10 + (uint32_t)(text[3]-'0');
	x = x *  6 + (uint32_t)(text[4]-'0');
	x = x * 10 + (uint32_t)(text[5]-'0');
	x = x * 1000;
	if (text[6] == '.') {
		if (text[7] != 0) {
			if (text[7] < '0' || text[7] > '9') return 0;
			x += (uint32_t)(text[7]-'0') * 100;
			if (text[8] != 0) {
				if (text[8] < '0' || text[8] > '9') return 0;
				x += (uint32_t)(text[8]-'0') * 10;
				if (text[9] != 0) {
					if (text[9] < '0' || text[9] > '9') return 0;
					x += (uint32_t)(text[9]-'0');
					if (text[10] != 0) {
						if (text[10] < '0' || text[10] > '9') return 0;
						x += text[10] > '5';
					}
				}
			}
		}
	}

	*result = x;
	return 1;
}

static int nmea_angle(const char *text, double *result)
{
	uint32_t x = 0;
	double v;
	int dotidx = (int)(strchrnul(text, '.') - text);

	switch(dotidx) {
	case 5:
		if (text[dotidx - 5] < '0' || text[dotidx - 5] > '9')
			return 0;
		x = x * 10 + (uint32_t)(text[dotidx - 5] - '0');
	case 4:
		if (text[dotidx - 4] < '0' || text[dotidx - 4] > '9')
			return 0;
		x = x * 10 + (uint32_t)(text[dotidx - 4] - '0');
	case 3:
		if (text[dotidx - 3] < '0' || text[dotidx - 3] > '9')
			return 0;
		x = x * 10 + (uint32_t)(text[dotidx - 3] - '0');
	case 2:
		v = atof(&text[dotidx - 2]);
		break;
	case 1:
		if (text[dotidx - 1] < '0' || text[dotidx - 1] > '9')
			return 0;
	case 0:
		v = atof(text);
		break;
	default:
		return 0;
	}

	*result = (double)x + v * 0.01666666666666666666666; /* 1 / 60 */

	return 1;
}

static int nmea_set(
		const char *tim,
		const char *lat, const char *latu,
		const char *lon, const char *lonu,
		const char *alt, const char *altu,
		const char *spe,
		const char *tra,
		const char *dat
)
{
	struct gps gps;

	DEBUG(afbitf, "time=%s latitude=%s%s longitude=%s%s altitude=%s%s speed=%s track=%s date=%s",
		tim, lat, latu, lon, lonu, alt, altu, spe, tra, dat);

	/* get the time in milliseconds */
	if (tim == NULL)
		gps.set.time = 0;
	else {
		if (!nmea_time(tim, &gps.time))
			return 0;
		gps.set.time = 1;
	}

	/* get the latitude */
	if (lat == NULL || latu == NULL)
		gps.set.latitude = 0;
	else {
		if ((latu[0] != 'N' && latu[0] != 'S') || latu[1] != 0)
			return 0;
		if (!nmea_angle(lat, &gps.latitude))
			return 0;
		if (latu[0] == 'S')
			gps.latitude = -gps.latitude;
		gps.set.latitude = 1;
	}

	/* get the longitude */
	if (lon == NULL || lonu == NULL)
		gps.set.longitude = 0;
	else {
		if ((lonu[0] != 'E' && lonu[0] != 'W') || lonu[1] != 0)
			return 0;
		if (!nmea_angle(lon, &gps.longitude))
			return 0;
		if (lonu[0] == 'W')
			gps.longitude = 360.0 - gps.longitude;
		gps.set.longitude = 1;
	}

	/* get the altitude */
	if (alt == NULL || altu == NULL)
		gps.set.altitude = 0;
	else {
		if (altu[0] != 'M' || altu[1] != 0)
			return 0;
		gps.altitude = atof(alt);
		gps.set.altitude = 1;
	}

	/* get the speed */
	if (spe == NULL)
		gps.set.speed = 0;
	else {
		gps.speed = atof(spe) * KNOT_TO_METER_PER_SECOND;
		gps.set.speed = 1;
	}

	/* get the track */
	if (tra != NULL)
		gps.set.track = 0;
	else {
		gps.track = atof(tra);
		gps.set.track = 1;
	}

	/* push the frame */
	frameidx = (frameidx ? : (int)(sizeof frames / sizeof *frames)) - 1; 
	frames[frameidx] = gps;
	newframes++;

	DEBUG(afbitf, "time:%d=%d latitude:%d=%g longitude:%d=%g altitude:%d=%g speed:%d=%g track:%d=%g",
		(int)gps.set.time, gps.set.time ? (int)gps.time : 0,
		(int)gps.set.latitude, gps.set.latitude ? gps.latitude : 0,
		(int)gps.set.longitude, gps.set.longitude ? gps.longitude : 0,
		(int)gps.set.altitude, gps.set.altitude ? gps.altitude : 0,
		(int)gps.set.speed, gps.set.speed ? gps.speed : 0,
		(int)gps.set.track, gps.set.track ? gps.track : 0
	);

	return 1;
}

static int nmea_split(char *s, char *fields[], int count)
{
	int index = 0;
	while (*s && index < count) {
		fields[index++] = s;
		while (*s && *s != ',')
			s++;
		if (*s == ',')
			*s++ = 0;
	}
	return !*s && index == count;
}

/*
 * interprete one sentence GGA - Fix information
 */
static int nmea_gga(char *s)
{
	char *f[14];

	return nmea_split(s, f, (int)(sizeof f / sizeof *f))
		&& *f[5] != '0'
		&&  nmea_set(f[0], f[1], f[2], f[3], f[4], f[6], f[7], NULL, NULL, NULL);
}

/*
 * interprete one sentence RMC - Recommended Minimum
 */
static int nmea_rmc(char *s)
{
	char *f[12];

	return nmea_split(s, f, (int)(sizeof f / sizeof *f))
		&& *f[1] == 'A'
		&&  nmea_set(f[0], f[2], f[3], f[4], f[5], NULL, NULL, f[6], f[7], f[8]);
}


/*
 * interprete one NMEA sentence
 */
static int nmea_sentence(char *s)
{
	if (!s[0] || !s[1])
		return 0;

	if (s[2] == 'G' && s[3] == 'G' && s[4] == 'A' && s[5] == ',')
		return nmea_gga(&s[6]);

	if (s[2] == 'R' && s[3] == 'M' && s[4] == 'C' && s[5] == ',')
		return nmea_rmc(&s[6]);

	return 0;
}

/*
 * reads the NMEA stream
 */
static int nmea_read(int fd)
{
	static char buffer[160];
	static int pos = 0;
	static int overflow = 0;

	int rc;

	for(;;) {
		rc = (int)read(fd, &buffer[pos], sizeof buffer - (size_t)pos);
		if (rc < 0) {
			/* its an error if not interrupted */
			if (errno != EINTR)
				return rc;
		} else if (rc == 0) {
			/* nothing more to be read */
			return 0;
		} else {
			/* scan the buffer */
			while (pos != rc) {
				if (buffer[pos] != '\n') {
					pos++;
					if (pos == rc) {
						if (pos == (int)(sizeof buffer)) {
							overflow = 1;
							pos = 0;
							rc = 0;
						}
					}
				} else {
					if (buffer[0] == '$' && pos > 0 && buffer[pos-1] == '\r' && !overflow) {
						if (pos > 3 && buffer[pos-4] == '*') {
							/* TODO: check the cheksum */
							buffer[pos-4] = 0;
						} else {
							buffer[pos-1] = 0;
						}
						nmea_sentence(&buffer[1]);
					}
					pos++;
					rc -= pos;
					if (rc > 0)
						memmove(buffer, buffer+pos, (size_t)rc);
					pos = 0;
					overflow = 0;
				}
			}
		}
	}
}

/*
 * called on an event on the NMEA stream
 */
static int nmea_on_event(sd_event_source *s, int fd, uint32_t revents, void *userdata)
{
	/* read available data */
	if ((revents & EPOLLIN) != 0) {
		nmea_read(fd);
		event_send();
	}

	/* check if error or hangup */
	if ((revents & (EPOLLERR|EPOLLRDHUP|EPOLLHUP)) != 0) {
		sd_event_source_unref(s);
		close(fd);
		nmea_connect(fd);
	}

	return 0;
}

/*
 * opens a socket to a host and a service (or port)
 */
static int open_socket_to(const char *host, const char *service)
{
	int rc, fd;
	struct addrinfo hint, *rai, *iai;

	/* get addr */
	memset(&hint, 0, sizeof hint);
	hint.ai_family = AF_INET;
	hint.ai_socktype = SOCK_STREAM;
	rc = getaddrinfo(host, service, &hint, &rai);
	if (rc != 0)
		return -1;

	/* get the socket */
	iai = rai;
	while (iai != NULL) {
		fd = socket(iai->ai_family, iai->ai_socktype, iai->ai_protocol);
		if (fd >= 0) {
			rc = connect(fd, iai->ai_addr, iai->ai_addrlen);
			if (rc == 0) {
				fcntl(fd, F_SETFL, O_NONBLOCK);
				freeaddrinfo(rai);
				return fd;
			}
			close(fd);
		}
		iai = iai->ai_next;
	}
	freeaddrinfo(rai);
	return -1;
}

/*
 * connection to nmea stream
 */
static int nmea_connect()
{
	sd_event_source *source;
	int rc, fd;
	const char *host;
	const char *service;

	/* TODO connect to somewhere else */
	host = "sinagot.net";
	service = "5001";
	fd = open_socket_to(host, service);
	if (fd < 0) {
		ERROR(afbitf, "can't connect to host %s, service %s", host, service);
		return fd;
	}

	/* adds to the event loop */
	rc = sd_event_add_io(afb_daemon_get_event_loop(afbitf->daemon), &source, fd, EPOLLIN, nmea_on_event, NULL);
	if (rc < 0) {
		close(fd);
		ERROR(afbitf, "can't coonect host %s, service %s to the event loop", host, service);
	}
	return rc;
}

/*
 * Returns the type corresponding to the given name
 */
static enum type type_of_name(const char *name)
{
	enum type result;
	if (name == NULL)
		return type_DEFAULT;
	for (result = 0 ; result != type_COUNT ; result++)
		if (strcmp(type_NAMES[result], name) == 0)
			return result;
	return type_INVALID;
}

/*
 * extract a valid type from the request
 */
static int get_type_for_req(struct afb_req req, enum type *type)
{
	if ((*type = type_of_name(afb_req_value(req, "type"))) != type_INVALID)
		return 1;
	afb_req_fail(req, "unknown-type", NULL);
	return 0;
}
	
/*
 * Get the last known position
 */
static void get(struct afb_req req)
{
	enum type type;
	if (get_type_for_req(req, &type))
		afb_req_success(req, position(type), NULL);
}

/*
 * subscribe to notification of position
 */
static void subscribe(struct afb_req req)
{
	enum type type;
	const char *period;
	struct event *event;
	struct json_object *json;

	if (get_type_for_req(req, &type)) {
		period = afb_req_value(req, "period");
		event = event_get(type, period == NULL ? 2000 : atoi(period));
		if (event == NULL)
			afb_req_fail(req, "out-of-memory", NULL);
		else if (afb_req_subscribe(req, event->event) != 0)
			afb_req_fail_f(req, "failed", "afb_req_subscribe returned an error: %m");
		else {
			json = json_object_new_object();
			json_object_object_add(json, "name", json_object_new_string(event->name));
			json_object_object_add(json, "id", json_object_new_int(event->id));
			afb_req_success(req, json, NULL);
		}
	}
}

/*
 * unsubscribe a previous subscription
 */
static void unsubscribe(struct afb_req req)
{
	const char *id;
	struct event *event;

	id = afb_req_value(req, "id");
	if (id == NULL)
		afb_req_fail(req, "missing-id", NULL);
	else {
		event = event_of_id(atoi(id));
		if (event == NULL)
			afb_req_fail(req, "bad-id", NULL);
		else {
			afb_req_unsubscribe(req, event->event);
			afb_req_success(req, NULL, NULL);
		}
	}
}

/*
 * array of the verbs exported to afb-daemon
 */
static const struct afb_verb_desc_v1 binding_verbs[] = {
  /* VERB'S NAME            SESSION MANAGEMENT          FUNCTION TO CALL         SHORT DESCRIPTION */
  { .name= "get",          .session= AFB_SESSION_NONE, .callback= get,          .info= "get the last known data" },
  { .name= "subscribe",    .session= AFB_SESSION_NONE, .callback= subscribe,    .info= "subscribe to notification of position" },
  { .name= "unsubscribe",  .session= AFB_SESSION_NONE, .callback= unsubscribe,  .info= "unsubscribe a previous subscription" },
  { .name= NULL } /* marker for end of the array */
};

/*
 * description of the binding for afb-daemon
 */
static const struct afb_binding binding_description =
{
  /* description conforms to VERSION 1 */
  .type= AFB_BINDING_VERSION_1,
  .v1= {			/* fills the v1 field of the union when AFB_BINDING_VERSION_1 */
    .prefix= "gps",		/* the API name (or binding name or prefix) */
    .info= "Access to the GPS data",	/* short description of of the binding */
    .verbs = binding_verbs	/* the array describing the verbs of the API */
  }
};

/*
 * activation function for registering the binding called by afb-daemon
 */
const struct afb_binding *afbBindingV1Register(const struct afb_binding_interface *itf)
{
	afbitf = itf;			/* records the interface for accessing afb-daemon */

	nmea_connect();

	return &binding_description;	/* returns the description of the binding */
}


