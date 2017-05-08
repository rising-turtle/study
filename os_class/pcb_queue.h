#ifndef PCB_QUEUE_H
#define PCB_QUEUE_H

#include <stdio.h>

struct saved_regs
{
  double reg1, reg2, reg32, reg_pc;
};

struct pcb_type
{
  int proc_id; 
  int proc_priority; 
  int proc_state;
  int proc_class;
  struct saved_regs * reg_pt; 
};

struct process_list
{
  struct pcb_type *pcb_pt;
  struct process_list * who_follows;
};

struct process_list *start = NULL;
int allocated_id = 1;
int allocated_prio = 1;
int allocated_class = 1;

struct saved_regs *alloc_saved_regs();
struct pcb_type * create_new_pcb();
void add_new_process(struct pcb_type *);


#endif
