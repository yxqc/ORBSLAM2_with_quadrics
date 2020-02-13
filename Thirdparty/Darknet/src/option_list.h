#ifndef OPTION_LIST_H
#define OPTION_LIST_H
#include "list.h"

typedef struct{
    char *key;
    char *val;
    int used;
} kvp;


int read_option(char *s, list_darknet *options);
void option_insert(list_darknet *l, char *key, char *val);
char *option_find(list_darknet *l, char *key);
float option_find_float(list_darknet *l, char *key, float def);
float option_find_float_quiet(list_darknet *l, char *key, float def);
void option_unused(list_darknet *l);

#endif
