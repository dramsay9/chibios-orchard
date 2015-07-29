#ifndef __ORCHARD_COMPASS_H__
#define __ORCHARD_COMPASS_H__

struct compass_data {
  long x;
  long y;
  long z;
};

void compassStart(I2CDriver *driver);
void compassStop(void);
msg_t compassPoll(struct compass_data *data);

#endif /* __ORCHARD_COMPASS_H__ */
