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
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>

#include <json-c/json.h>

#include <systemd/sd-event.h>

#include <afb/afb-binding.h>

#define NAUTICAL_MILE_IN_METER     1852
#define KNOT_TO_METER_PER_SECOND   0.5144444444444444

/*
 * references:
 *
 *       https://www.w3.org/TR/geolocation-API/
 *       http://www.gpsinformation.org/dale/nmea.htm
 */

/* declare the connection routine */
static int nmea_connect();

/*
 * the interface to afb-daemon
 */
const struct afb_binding_interface *afbitf;

struct gps {
	unsigned time_is_set: 1;
	unsigned latitude_is_set: 1;
	unsigned longitude_is_set: 1;
	unsigned altitude_is_set: 1;
	unsigned speed_is_set: 1;
	unsigned track_is_set: 1;

	uint32_t time;
	double latitude;
	double longitude;
	double altitude;
	double speed;
	double track;
};

static struct gps frames[10];
static int frameidx;
static int newframes;

static struct json_object *last_position;

static struct json_object *position()
{
	if (newframes) {
		newframes = 0;
	}
	return last_position;
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
		if (text[dotidx - 2] < '0' || text[dotidx - 2] > '9')
			return 0;
		x = x * 6 + (uint32_t)(text[dotidx - 3] - '0');
	case 1:
		if (text[dotidx - 1] < '0' || text[dotidx - 1] > '9')
			return 0;
		x = x * 10 + (uint32_t)(text[dotidx - 3] - '0');
	case 0:
		break;
	default:
		return 0;
	}

	if (text[dotidx] == '.')
		*result = atof(&text[dotidx]) + x;
	else
		*result = x;

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
		gps.time_is_set = 0;
	else {
		if (!nmea_time(tim, &gps.time))
			return 0;
		gps.time_is_set = 1;
	}

	/* get the latitude */
	if (lat == NULL || latu == NULL)
		gps.latitude_is_set = 0;
	else {
		if ((latu[0] != 'N' && latu[0] != 'S') || latu[1] != 0)
			return 0;
		if (!nmea_angle(lat, &gps.latitude))
			return 0;
		if (latu[0] == 'S')
			gps.latitude = -gps.latitude;
		gps.latitude_is_set = 1;
	}

	/* get the longitude */
	if (lon == NULL || lonu == NULL)
		gps.longitude_is_set = 0;
	else {
		if ((lonu[0] != 'E' && lonu[0] != 'W') || lonu[1] != 0)
			return 0;
		if (!nmea_angle(lon, &gps.longitude))
			return 0;
		if (lonu[0] == 'W')
			gps.longitude = 360.0 - gps.longitude;
		gps.longitude_is_set = 1;
	}

	/* get the altitude */
	if (alt == NULL || altu == NULL)
		gps.altitude_is_set = 0;
	else {
		if (altu[0] != 'M' || altu[1] != 0)
			return 0;
		gps.altitude = atof(alt);
		gps.altitude_is_set = 1;
	}

	/* get the speed */
	if (spe == NULL)
		gps.speed_is_set = 0;
	else {
		gps.speed = atof(spe) * KNOT_TO_METER_PER_SECOND;
		gps.speed_is_set = 1;
	}

	/* get the track */
	if (tra != NULL)
		gps.track_is_set = 0;
	else {
		gps.track = atof(tra);
		gps.track_is_set = 1;
	}

	/* push the frame */
	frameidx = (frameidx ? : (int)(sizeof frames / sizeof *frames)) - 1; 
	frames[frameidx] = gps;
	newframes++;

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
	if ((revents & EPOLLIN) != 0)
		nmea_read(fd);

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
 * Get the last known position
 */
static void get(struct afb_req req)
{
	afb_req_success(req, position(), NULL);
}

/*
 * subscribe to notification of position
 */
static void subscribe(struct afb_req req)
{
	afb_req_success(req, NULL, NULL);
}

/*
 * unsubscribe a previous subscription
 */
static void unsubscribe(struct afb_req req)
{
	afb_req_success(req, NULL, NULL);
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


