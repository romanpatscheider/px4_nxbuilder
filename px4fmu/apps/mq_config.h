//Configuration for mq beween different apps

#ifndef MQ_CONFIG_H_
#define MQ_CONFIG_H_

const struct mq_attr MQ_ATTR_GPS = { .mq_flags = 0, .mq_maxmsg = 10, .mq_msgsize = 2, .mq_curmsgs = 0 };

#endif /* MQ_CONFIG_H_ */
