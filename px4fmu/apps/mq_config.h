//Configuration for mq beween different apps

#ifndef MQ_CONFIG_H_
#define MQ_CONFIG_H_

#define MQ_ATTR_GPS { .mq_flags = 0, .mq_maxmsg = 10, .mq_msgsize = 2, .mq_curmsgs = 0 }
#define MQ_NAME_GPS "gps_queue"

#endif /* MQ_CONFIG_H_ */
