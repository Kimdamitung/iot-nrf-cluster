#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include "mqtt.h"
#include "posix_sockets.h"
#include "gpio/gpio.h"

#define BROKER  "demo.thingsboard.io"
#define PORT    "1883"
#define TOKEN   "KzkFxGtN076bsPJz3Lvw"
#define TOPIC   "v1/devices/me/telemetry"
#define SUB     "v1/devices/me/attributes"

volatile sig_atomic_t  isTrue = 1;

void handle(int sig){
    isTrue = 0;
}

void subcriber_callback(void** unused, struct mqtt_response_publish *published);
void* client_refresher(void* client);
void exit_example(int status, int sockfd, pthread_t *client_daemon);

int main(int argc, const char *argv[]){
    signal(SIGINT, handle);
    GPIO.gpio_export(GPIO4_C1_Z);
    GPIO.gpio_export(GPIO4_C0_Z);
    GPIO.gpio_export(GPIO4_A4_D);
    GPIO.gpio_export(GPIO4_A3_D);
    GPIO.gpio_export(GPIO4_A2_D);
    GPIO.gpio_export(GPIO4_A6_D);
    GPIO.gpio_export(GPIO4_B0_D);
    GPIO.gpio_export(GPIO4_B1_D);
    GPIO.gpio_mode(GPIO4_C1_Z, OUTPUT);
    GPIO.gpio_mode(GPIO4_C0_Z, OUTPUT);
    GPIO.gpio_mode(GPIO4_A4_D, OUTPUT);
    GPIO.gpio_mode(GPIO4_A3_D, OUTPUT);
    GPIO.gpio_mode(GPIO4_A2_D, OUTPUT);
    GPIO.gpio_mode(GPIO4_A6_D, OUTPUT);
    GPIO.gpio_mode(GPIO4_B0_D, OUTPUT);
    GPIO.gpio_mode(GPIO4_B1_D, OUTPUT);
    const char* addr;
    const char* port;
    const char* topic;
    addr = (argc > 1) ? argv[1] : BROKER;
    port = (argc > 2) ? argv[2] : PORT;
    topic = (argc > 3) ? argv[3] : TOPIC;
    int sockfd = open_nb_socket(addr, port);
    if(sockfd == -1){
        perror("Failed to open socket: ");
        exit_example(EXIT_FAILURE, sockfd, NULL);
    }
    struct mqtt_client client;
    uint8_t sendbuf[2048];
    uint8_t recvbuf[1024];
    mqtt_init(&client, sockfd, sendbuf, sizeof(sendbuf), recvbuf, sizeof(recvbuf), subcriber_callback);
    const char* client_id = "luckfox";
    uint8_t connect_flags = MQTT_CONNECT_CLEAN_SESSION;
    mqtt_connect(&client, client_id, NULL, NULL, 0, TOKEN, NULL, connect_flags, 400);
    if (client.error != MQTT_OK) {
        fprintf(stderr, "error: %s\n", mqtt_error_str(client.error));
        exit_example(EXIT_FAILURE, sockfd, NULL);
    }
    pthread_t client_daemon;
    if(pthread_create(&client_daemon, NULL, client_refresher, &client)) {
        fprintf(stderr, "Failed to start client daemon.\n");
        exit_example(EXIT_FAILURE, sockfd, NULL);
    }
    mqtt_subscribe(&client, SUB, 0);
    float i = 0.0;
    while (isTrue) {
        if(i > 20)
            i = 0.0;
        float temperature = 25.5 + i; 
        float humidity    = 60.2 - i;
        char application_message[256];
        snprintf(application_message, sizeof(application_message), "{\"temperature\":%.2f, \"humidity\":%.2f}", temperature, humidity);
        printf("%s published : \"%s\"\n", argv[0], application_message);
        mqtt_publish(&client, topic, application_message, strlen(application_message), MQTT_PUBLISH_QOS_0);
        if (client.error != MQTT_OK) {
            fprintf(stderr, "error: %s\n", mqtt_error_str(client.error));
            exit_example(EXIT_FAILURE, sockfd, &client_daemon);
        }
        usleep(5000 * 1000);
        i++;
    }
    GPIO.gpio_unexport(GPIO4_C1_Z);
    GPIO.gpio_unexport(GPIO4_C0_Z);
    GPIO.gpio_unexport(GPIO4_A4_D);
    GPIO.gpio_unexport(GPIO4_A3_D);
    GPIO.gpio_unexport(GPIO4_A2_D);
    GPIO.gpio_unexport(GPIO4_A6_D);
    GPIO.gpio_unexport(GPIO4_B0_D);
    GPIO.gpio_unexport(GPIO4_B1_D);
    return 0;
}

void exit_example(int status, int sockfd, pthread_t *client_daemon){
    if (sockfd != -1) close(sockfd);
    if (client_daemon != NULL) pthread_cancel(*client_daemon);
    exit(status);
}

void subcriber_callback(void** unused, struct mqtt_response_publish *published) {
    char* topic_name = (char*) malloc(published->topic_name_size + 1);
    memcpy(topic_name, published->topic_name, published->topic_name_size);
    topic_name[published->topic_name_size] = '\0';
    char* payload = (char*) malloc(published->application_message_size + 1);
    memcpy(payload, published->application_message, published->application_message_size);
    payload[published->application_message_size + 1] = '\0';
    printf("Received publish('%s'): %s\n", topic_name, payload);
    if(strstr(payload, "\"value\":true")){
        GPIO.gpio_level(GPIO4_C1_Z, LOW);
        GPIO.gpio_level(GPIO4_C0_Z, LOW);
        GPIO.gpio_level(GPIO4_A4_D, LOW);
        GPIO.gpio_level(GPIO4_A3_D, LOW);
        GPIO.gpio_level(GPIO4_A2_D, LOW);
        GPIO.gpio_level(GPIO4_A6_D, LOW);
        GPIO.gpio_level(GPIO4_B0_D, LOW);
        GPIO.gpio_level(GPIO4_B1_D, LOW);
    }else if (strstr(payload, "\"value\":false")){
        GPIO.gpio_level(GPIO4_C1_Z, HIGH);
        GPIO.gpio_level(GPIO4_C0_Z, HIGH);
        GPIO.gpio_level(GPIO4_A4_D, HIGH);
        GPIO.gpio_level(GPIO4_A3_D, HIGH);
        GPIO.gpio_level(GPIO4_A2_D, HIGH);
        GPIO.gpio_level(GPIO4_A6_D, HIGH);
        GPIO.gpio_level(GPIO4_B0_D, HIGH);
        GPIO.gpio_level(GPIO4_B1_D, HIGH);
    }
    free(topic_name);
    free(payload);
}

void* client_refresher(void* client){
    while(1){
        mqtt_sync((struct mqtt_client*) client);
        usleep(100000U);
    }
    return NULL;
}