#ifndef __BQ2022A_H__
#define __BQ2022A_H__

struct bq2022a_platform_data {
	spinlock_t bqlock;
};

extern int bq2022a_read_bat_id(void);
extern int bq2022a_read_bat_module_id(void);
#endif
