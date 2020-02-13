#ifndef LIST_H
#define LIST_H
#include "darknet.h"

list_darknet *make_list();
int list_find(list_darknet *l, void *val);

void list_insert(list_darknet *, void *);


void free_list_contents(list_darknet *l);

#endif
