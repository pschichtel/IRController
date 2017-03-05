#include <stdio.h>

#include "gpio/common.h"
#include "gpio/pwm.h"
#include "gpio/event.h"
#include <signal.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define flip(var, a, b) var = (var == (a) ? (b) : (a))
#define is_low(x) (x)==LOW
#define is_high(x) (x)==HIGH

bool keep_spinning = true;

char* value_names[] = {
        "LOW",
        "HIGH"
};

char* edge_names[] = {
        "NONE",
        "RISING",
        "FALLING",
        "BOTH"
};

void handleInterrupt(int signal) {
    keep_spinning = false;
}

void timespec_diff(struct timespec *earlier, struct timespec *later, struct timespec *delta)
{
    if ((later->tv_nsec - earlier->tv_nsec) < 0) {
        delta->tv_sec = later->tv_sec - earlier->tv_sec - 1;
        delta->tv_nsec = later->tv_nsec - earlier->tv_nsec + 1000000000;
    } else {
        delta->tv_sec = later->tv_sec - earlier->tv_sec;
        delta->tv_nsec = later->tv_nsec - earlier->tv_nsec;
    }
}

struct ir_signal {
    unsigned int value;
    struct timespec* duration;
};

struct ir_recording {
    struct ir_signal** signals;
    size_t length;
};

struct list {
    void* value;
    struct list* tail;
};

void free_list(struct list* head) {
    struct list* next = NULL;
    while (head != NULL) {
        next = head->tail;
        free(head);
        head = next;
    }
}

void free_recorded_signals(struct ir_recording* recording) {
    struct ir_signal* current;
    for (size_t i = 0; i < recording->length; ++i) {
        current = recording->signals[i];
        free(current->duration);
        free(current);
    }
    free(recording);
}

size_t list_length(struct list* head) {
    size_t length = 0;
    while (head != NULL) {
        length++;
        head = head->tail;
    }
    return length;
}

void** list_to_array_with_length(struct list* head, size_t length) {
    void** array = malloc(length * sizeof(void*));
    size_t i = 0;
    while (head != NULL && i < length) {
        array[i++] = head->value;
        head = head->tail;
    }
    return array;
}

void** list_to_array(struct list* head, size_t* length) {
    *length = list_length(head);
    return list_to_array_with_length(head, *length);
}

void array_reverse(void **array, size_t length) {
    void* tmp;

    for (size_t i = 0, j = length - 1; i < j; ++i, --j) {
        tmp = array[i];
        array[i] = array[j];
        array[j] = tmp;
    }
}

struct ir_recording* record_ir_signal(int pin, int timeout) {

    if (gpio_export(pin) == -1) {
        printf("Error exporting pin: %s\n", get_error_msg());
        return NULL;
    }
    if (gpio_set_direction(pin, INPUT) == -1) {
        printf("Error configuring pin as input: %s\n", get_error_msg());
        return NULL;
    }

    // The input signal received by the IR receiver is inverted, meaning:
    //  no signal -> value = HIGH
    //  signal    -> value = LOW
    // and as such:
    //  falling edge -> signal starts
    //  rising edge  -> signal ends

    unsigned int value;
    gpio_get_value(pin, &value);

    // wait until a stable HIGH state is reached (no signal coming in)
    while (is_low(value)) {
        printf("we are still low (receiving a signal)\n");
        blocking_wait_for_edge(pin, RISING_EDGE);
        printf("rising edge!\n");
        // wait a second to see of the receiver is busy
        int r = blocking_wait_for_edge_with_timeout(pin, FALLING_EDGE, 1000);
        printf("falling edge? -> %d\n", r);

        gpio_get_value(pin, &value);
    }
    printf("Reached stable start state, recording now...\n");

    // wait until first signal
    blocking_wait_for_edge(pin, FALLING_EDGE);

    struct list* signal_list = NULL;

    struct timespec current_time;
    struct timespec next_time;
    struct timespec delta;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    unsigned int current_value = LOW;
    unsigned int next_value;
    unsigned int wait_for = RISING_EDGE;

    while (keep_spinning) {
        // Block until next edge. if the next edge will be falling (new signal begins) use the timeout.
        // This way, running signals be read completely, but we don't hang on the last one.
        blocking_wait_for_edge_with_timeout(pin, wait_for, wait_for == FALLING_EDGE ? timeout : -1);
        gpio_get_value(pin, &next_value);
        if (current_value == next_value) {
            // Wait for edge timed out
            break;
        }

        clock_gettime(CLOCK_MONOTONIC, &next_time);
        timespec_diff(&current_time, &next_time, &delta);

        flip(wait_for, RISING_EDGE, FALLING_EDGE);

        struct ir_signal* signal = malloc(sizeof(struct ir_signal));
        signal->value = is_high(current_value) ? LOW : HIGH;
        signal->duration = malloc(sizeof(struct timespec));
        memcpy(signal->duration, &delta, sizeof(struct timespec));

        struct list* entry = malloc(sizeof(struct list));
        entry->value = signal;
        entry->tail = signal_list;
        signal_list = entry;

        current_value = next_value;
        current_time = next_time;
    }

    size_t length;
    struct ir_signal** signals = (struct ir_signal **) list_to_array(signal_list, &length);
    free_list(signal_list);
    array_reverse((void **) signals, length);

    struct ir_recording* recorded = malloc(sizeof(struct ir_recording));
    recorded->signals = signals;
    recorded->length = length;

    // cleanup
    gpio_event_remove(pin);
    gpio_unexport(pin);

    return recorded;
}

void repeat_ir_signal(struct ir_recording* recorded_signals) {

    struct timespec start;
    struct timespec left;

    struct ir_signal** signals = recorded_signals->signals;
    struct ir_signal* current;

    pwm_start("PWM0", 0.0f, 38600.0f, 1);
    for (size_t i = 0; i < recorded_signals->length; ++i)
    {
        current = signals[i];
        printf("Setting %s for %li.%li\n", value_names[current->value], current->duration->tv_sec, current->duration->tv_nsec);
        pwm_set_duty_cycle("PWM0", is_high(current->value) ? 50.0f : 0.0f);
        start.tv_sec = current->duration->tv_sec;
        start.tv_nsec = current->duration->tv_nsec;
        nanosleep(&start, &left);
    }
    pwm_disable("PWM0");
    pwm_cleanup();
}

int main(int argc, char** args) {

    signal(SIGINT, handleInterrupt);
    signal(SIGTERM, handleInterrupt);

    int pin = lookup_gpio_by_name("AP-EINT1");
    if (pin == -1) {
        printf("Failed to lookup pin!\n");
        return 1;
    }
    printf("PIN: %d\n", pin);
    gpio_unexport(pin);

    struct ir_recording* recording = NULL;

    while (keep_spinning) {
        recording = record_ir_signal(pin, 2000);
        if (recording != NULL) {
            printf("Signal of length %zu recorded, replaying...\n", recording->length);
            repeat_ir_signal(recording);
            printf("Signal replayed.\n");
            free_recorded_signals(recording);
        } else {
            printf("Nothing recorded!\n");
        }
    }

    return 0;
}

