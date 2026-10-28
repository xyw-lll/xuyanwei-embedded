#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H



typedef struct {
    void (*on)(void);
    void (*off)(void);
    void (*toggle)(void);
}BuzzerInterface_t,*pBuzzeInterface_t;

extern BuzzerInterface_t UserBuzzer;

#endif /* __BSP_BUZZER_H */

