#include "pcb_queue.h"
#include <stdlib.h>

struct saved_regs * alloc_saved_regs()
{
  struct saved_regs * ret = (struct saved_regs*)malloc(sizeof(struct saved_regs));
  return ret;
}

struct pcb_type * create_new_pcb()
{
  struct pcb_type * ret = (struct pcb_type *)malloc(sizeof(struct pcb_type));
  
  ret->reg_pt = alloc_saved_regs();
  ret->proc_id = allocated_id++;
  ret->proc_priority = allocated_prio++ ;
  ret->proc_class = allocated_class++ ;
  return ret;
}

void add_new_process(struct pcb_type * new_pcb)
{
  struct process_list * new_proc = (struct process_list*)malloc(sizeof(struct process_list));
  new_proc->pcb_pt = new_pcb;
  new_proc->who_follows = start;
  start = new_proc;
}

void print_p(struct process_list* p, int proc_index)
{
  struct pcb_type* p_t = p->pcb_pt;
  printf("index of process: %d\n", proc_index);
  printf("addr_pcb = %p\n", p_t);
  printf("proc_id = %d\n", p_t->proc_id);
  printf("proc_class = %d\n", p_t->proc_class);
  printf("proc_state = %d\n", p_t->proc_state);
}

int main(int argc, char* argv[])
{
  int N = 12;
  int i;
  for(i=0; i<N; i++)
  {
    struct pcb_type * new_pcb =  create_new_pcb();
    add_new_process(new_pcb);
  }

  struct process_list* pl = start;
  int proc_index = 0;
  while(pl != NULL)
  {
    print_p(pl, proc_index++);
    pl = pl->who_follows;
  }
  return 0;
}

