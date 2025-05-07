// Posture Controller

#define POS_PID_KP 50
#define POS_PID_KI 0.05
#define POS_PID_KD 0.01

#define POS_PID_MAX_KI 10000

#define ANGULAR_VELOCITY_MAX 10

typedef struct pos_pidf_s {
    float P;
    float I;
    float D;
} pos_pidf_t;

typedef struct pos_pidAxisData_s {
    float P;
    float I;
    float D;
    float Sum;
    float previous_error;
} pos_pidAxisData_t;