/*******************************************************************************
 * Copyright (c) 2019 Fabrizio Cabaleiro
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/

#include <linux/limits.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <string.h>
#include <errno.h>

#include <ncurses.h>

#include <sched.h>
#include <sys/shm.h>

#include <sys/time.h>

#define NTASKS  4
#define COUNTER 10000

/* avoid -Wimplicit-function-declaration */
extern int pthread_getattr_np(pthread_t, pthread_attr_t*);
extern int sched_getcpu();

#define XSTR(s) #s
#define STR(s) XSTR(s)

/* HW addresses */
#define  CSR_CYCLE       0xC00
#define  CSR_TIME        0xC01
#define  CSR_INSTRET     0xC02
#define  CSR_MHARTID     0xF14

#define MRW_MHPMEVENT3   0x323
#define MRW_MHPMEVENT4   0x324
#define MRW_MHPMCOUNTER3 0xB03
#define MRW_MHPMCOUNTER4 0xB04

/* Event Selector */
/* Event Class Instruction Commit Events */
#define EC_ICE 0x00
#define PMER_ICE_EXCEPTION_TAKEN                            ((1 <<  8) | EC_ICE)
#define PMER_ICE_INTEGER_LOAD_INSTRUCTION_RETIRED           ((1 <<  9) | EC_ICE)
#define PMER_ICE_INTEGER_STORE_INSTRUCTION_RETIRED          ((1 << 10) | EC_ICE)
#define PMER_ICE_ATOMIC_MEMORY_OPERATION_RETIRED            ((1 << 11) | EC_ICE)
#define PMER_ICE_SYSTEM_INSTRUCTION_RETIRED                 ((1 << 12) | EC_ICE)
#define PMER_ICE_INTEGER_ARITHMETIC_INSTRUCTION_RETIRED     ((1 << 13) | EC_ICE)
#define PMER_ICE_CONDITIONAL_BRANCH_RETIRED                 ((1 << 14) | EC_ICE)
#define PMER_ICE_JAL_INSTRUCTION_RETIRED                    ((1 << 15) | EC_ICE)
#define PMER_ICE_JALR_INSTRUCTION_RETIRED                   ((1 << 16) | EC_ICE)
#define PMER_ICE_INTEGER_MULTIPLICATION_INSTRUCTION_RETIRED ((1 << 17) | EC_ICE)
#define PMER_ICE_INTEGER_DIVISION_INSTRUCTION_RETIRED       ((1 << 18) | EC_ICE)

/* Event Class Microarchitectural Events */
#define EC_ME 0x01
#define PMER_ME_LOAD_USE_INTERLOCK                          ((1 <<  8) | EC_ME)
#define PMER_ME_LONG_LATENCY_INTERLOCK                      ((1 <<  9) | EC_ME)
#define PMER_ME_CSR_READ_INTERLOCK                          ((1 << 10) | EC_ME)
#define PMER_ME_INSTRUCTION_CACHE_ITIM_BUSY                 ((1 << 11) | EC_ME)
#define PMER_ME_DATA_CACHE_DTIM_BUSY                        ((1 << 12) | EC_ME)
#define PMER_ME_BRANCH_DIRECTION_MISPREDICTION              ((1 << 13) | EC_ME)
#define PMER_ME_BRANCH_JUMP_TARGET_MISPREDICTION            ((1 << 14) | EC_ME)
#define PMER_ME_PIPELINE_FLUSH_FROM_CSR_WRITE               ((1 << 15) | EC_ME)
#define PMER_ME_PIPELINE_FLUSH_FROM_OTHER_EVENT             ((1 << 16) | EC_ME)
#define PMER_ME_INTEGER_MULTIPLICATION_INTERLOCK            ((1 << 17) | EC_ME)

/* Event Class Memory System Events */
#define EC_MSE 0x02
#define PMER_MSE_INSTRUCTION_CACHE_MISS                     (( 1 << 8) | EC_MSE)
#define PMER_MSE_MEMORY_MAPPED_I_O_ACCESS                   (( 1 << 9) | EC_MSE)

/* When the first thread/job finishes, epoch1 gets set, then when the last
 * thread/job finishes epoch2 gets set.  Finally "epoch2 - epoch1" is used to
 * know if there was a thread/job that run slower than the rest. */
time_t epoch1, epoch2;

/* Transform command line argument to PMER */
unsigned long long int get_pmer_from_str(char *s);

/* Thread/Jobs arguments */
typedef struct {
    unsigned char done;
    unsigned int loops;
    unsigned int counter;
    long long int i;
    pid_t pid;
    unsigned long long int cycle;
    unsigned long long int time;
    unsigned long long int instret;
    unsigned long long int hpmcounter3;
    unsigned long long int hpmevent3;
    int coreid;
    void *stack_addr;
    size_t stack_size;
} targs_t;

/* Way to notify main task when jobs/thread are done */
char keep_waiting(targs_t *pta);

/* Command line arguments */
typedef struct {
    unsigned int loops;
    unsigned int counter;
    unsigned long long int hpmevent3;
    char fname[PATH_MAX];
} args_t;


/* Command line argument parser */
args_t args_get(int, char **);

/* The actual task that we want to monitor */
void* dummy_task(void *arg)
{
    unsigned long long int cycle = 0, pcycle = 0;
    unsigned long long int time = 0, ptime = 0;
    unsigned long long int instret = 0, pinstret = 0;
    unsigned long long int hpmcounter3 = 0, phpmcounter3 = 0;
    unsigned long long int hpmevent3 = 0;
    pthread_attr_t attr;
    int i, counter, counter_start, err;
    unsigned int loops;


    targs_t *pta = (targs_t*)arg;
    pta->pid = getpid();
    loops = pta->loops;
    counter = counter_start = pta->counter ? pta->counter : COUNTER;
    hpmevent3 = pta->hpmevent3;

    /* Get stack address and size */
    if((err = pthread_getattr_np(pthread_self(), &attr)))
    {
        fprintf(stderr, "pthread_getattr_np returned %d\n", err);
        pthread_exit((void*)EXIT_FAILURE);
    }
    if((err = pthread_attr_getstack(&attr, &pta->stack_addr, &pta->stack_size)))
    {
        fprintf(stderr, "pthread_attr_getstack returned %d\n", err);
        pthread_exit((void*)EXIT_FAILURE);
    }

    /* Initialize HW Performance Monitor counter/event */
    asm("csrrw zero, " STR(MRW_MHPMCOUNTER3) ", %0\n"
        :: "r" (hpmcounter3));
    asm("csrrw zero, " STR(MRW_MHPMEVENT3) ", %0\n"
        :: "r" (hpmevent3));

    /* Loop, most of the time should increment i, decreament counter and
     * compare to zero, so it should mostly load/store to/from stack, bnze and
     * bltu */
    for(i = 0; i < loops; i++)
    {
        if(counter == 0)
        {
            pta->i = i;
            pta->coreid = sched_getcpu();
            counter = counter_start;
            asm("csrrs %0, " STR(CSR_CYCLE)        ", zero\n"
                "csrrs %1, " STR(CSR_TIME)         ", zero\n"
                "csrrs %2, " STR(CSR_INSTRET)      ", zero\n"
                "csrrs %3, " STR(MRW_MHPMCOUNTER3) ", zero\n"
                : "=rm" (cycle), "=rm" (time), "=rm" (instret),
                  "=rm" (hpmcounter3));

            pta->cycle       = cycle       - pcycle;
            pta->time        = time        - ptime;
            pta->instret     = instret     - pinstret;
            pta->hpmcounter3 = hpmcounter3 - phpmcounter3;

            pcycle       = cycle;
            ptime        = time;
            pinstret     = instret;
            phpmcounter3 = hpmcounter3;
        }
        counter -= 1;
    }
    pta->done = 1;
    return (void*)NULL;
}

int main(int argc, char **argv)
{
    FILE *fout = NULL;
    char msg[512] = {0};
    char header[512] = {0};
    args_t args = {0};
#ifdef FORK
    const key_t ipcKey = time(NULL)%3600;
    const int perm = 0666;
    int shmId;
#else
    pthread_t tid[NTASKS];
#endif

    targs_t *pta;
    int i;

    args = args_get(argc, argv);
    if(args.loops == 0)
    {
        fprintf(stderr, "Need to set number of loops\n");
        return EXIT_FAILURE;
    }
    if(*args.fname)
    {
        if((fout = fopen(args.fname, "w")) == NULL)
        {
            fprintf(stderr, "Error fopen: '%s' %s\n", args.fname,
                    strerror(errno));
            return EXIT_FAILURE;
        }
    }
#ifdef FORK
    if((shmId = shmget(ipcKey, 4 * sizeof(targs_t), IPC_CREAT|perm)) == -1)
    {
        fprintf(stderr, "Error shmget: %s\n", strerror(errno));
        return EXIT_FAILURE;
    }
    if((pta = shmat(shmId, NULL, 0)) == (void*)-1)
    {
        fprintf(stderr, "Error shmat: %s\n", strerror(errno));
        return EXIT_FAILURE;
    }
#else
    pta = (targs_t*)malloc(NTASKS * sizeof(*pta));
#endif
    for(i = 0; i < NTASKS; i++)
    {
        pta[i].pid = 0;
        pta[i].loops = args.loops;
        pta[i].counter = args.counter;
        pta[i].hpmevent3 = args.hpmevent3;
        pta[i].i = 0;
        pta[i].done = 0;
    }
    initscr();
#ifdef FORK
    for(i = 0; i < NTASKS; i++)
    {
        if(fork() == 0)
        {
            dummy_task(&pta[i]);
            return EXIT_SUCCESS;
        }
    }
#else
    for(i = 0; i < NTASKS; i++)
    {
        pthread_create(&tid[i], NULL, dummy_task,
                       (void*)&pta[i]);
    }
#endif
    sprintf(header, "%6s "
                    "%5s "
                    "%9s "
                    "%22s "
                    "%22s "
                    "\n"
                    "%22s "
                    "%22s "
                    "%22s "
                    "%22s "
                    "%22s ",
                    "P.CPU",
                    "CPU",
                    "PID",
                    "Stack Ptr",
                    "Stack Size",
                    "cycle",
                    "time",
                    "instret",
                    "hpmc3",
                    "i");
    mvprintw(0, 0, header);
    if(fout)
        fprintf(fout, "%s\n", header);

    while(keep_waiting(pta))
    {
        for(i = 0; i < NTASKS; i++)
            if(pta[i].pid == 0)
                continue;
        for(i = 0; i < NTASKS; i++)
        {
            bzero(msg, sizeof(msg));
            sprintf(msg, "%6d "
                         "%5d "
                         "%9d "
                         "%22p "
                         "%22lu "
                         "\n"
                         "%22llu "
                         "%22llu "
                         "%22llu "
                         "%22llu "
                         "%22llu ",
                         sched_getcpu(),
                         pta[i].coreid,
                         pta[i].pid,
                         pta[i].stack_addr,
                         pta[i].stack_size,
                         pta[i].cycle,
                         pta[i].time,
                         pta[i].instret,
                         pta[i].hpmcounter3,
                         pta[i].i);
            mvprintw(i * 3 + 3, 0, msg);
            if(fout)
                fprintf(fout, "%s %d\n", msg, i);
        }
        refresh();
        usleep(100000);
    }
    epoch2 = time(NULL);
    endwin();
    if(fout)
        fclose(fout);
    return (epoch2 < (epoch1 + 5)) ? EXIT_SUCCESS : EXIT_FAILURE;
}

#define CMP_N_RETURN(x) if(!strcmp(s, #x)) return x;
unsigned long long int get_pmer_from_str(char *s)
{
    CMP_N_RETURN(PMER_ICE_EXCEPTION_TAKEN);
    CMP_N_RETURN(PMER_ICE_INTEGER_LOAD_INSTRUCTION_RETIRED);
    CMP_N_RETURN(PMER_ICE_INTEGER_STORE_INSTRUCTION_RETIRED);
    CMP_N_RETURN(PMER_ICE_ATOMIC_MEMORY_OPERATION_RETIRED);
    CMP_N_RETURN(PMER_ICE_SYSTEM_INSTRUCTION_RETIRED);
    CMP_N_RETURN(PMER_ICE_INTEGER_ARITHMETIC_INSTRUCTION_RETIRED);
    CMP_N_RETURN(PMER_ICE_CONDITIONAL_BRANCH_RETIRED);
    CMP_N_RETURN(PMER_ICE_JAL_INSTRUCTION_RETIRED);
    CMP_N_RETURN(PMER_ICE_JALR_INSTRUCTION_RETIRED);
    CMP_N_RETURN(PMER_ICE_INTEGER_MULTIPLICATION_INSTRUCTION_RETIRED);
    CMP_N_RETURN(PMER_ICE_INTEGER_DIVISION_INSTRUCTION_RETIRED);
    CMP_N_RETURN(PMER_ME_LOAD_USE_INTERLOCK);
    CMP_N_RETURN(PMER_ME_LONG_LATENCY_INTERLOCK);
    CMP_N_RETURN(PMER_ME_CSR_READ_INTERLOCK);
    CMP_N_RETURN(PMER_ME_INSTRUCTION_CACHE_ITIM_BUSY);
    CMP_N_RETURN(PMER_ME_DATA_CACHE_DTIM_BUSY);
    CMP_N_RETURN(PMER_ME_BRANCH_DIRECTION_MISPREDICTION);
    CMP_N_RETURN(PMER_ME_BRANCH_JUMP_TARGET_MISPREDICTION);
    CMP_N_RETURN(PMER_ME_PIPELINE_FLUSH_FROM_CSR_WRITE);
    CMP_N_RETURN(PMER_ME_PIPELINE_FLUSH_FROM_OTHER_EVENT);
    CMP_N_RETURN(PMER_ME_INTEGER_MULTIPLICATION_INTERLOCK);
    CMP_N_RETURN(PMER_MSE_INSTRUCTION_CACHE_MISS);
    CMP_N_RETURN(PMER_MSE_MEMORY_MAPPED_I_O_ACCESS);
    fprintf(stderr, "Error PMER argument not recognize '%s'\n", s);
    exit(EXIT_FAILURE);
}

char keep_waiting(targs_t *pta)
{
    int i;
    int done = 0;
    for(i = 0; i < NTASKS; i++)
        done += pta[i].done;
    if(done != 0 && epoch1 == 0)
    {
        epoch1 = time(NULL);
    }
    return done != NTASKS;
}

args_t args_get(int argc, char **argv)
{
    args_t a = {0};
    char opt;
    while((opt = getopt(argc, argv, "l:c:3:o:")) != (char)-1)
    {
        switch(opt)
        {
            case 'l': /* loops */
                a.loops = atoi(optarg);
                break;
            case 'c': /* counter */
                a.counter = atoi(optarg);
                break;
            case '3': /* HPM 3 */
                a.hpmevent3 |= get_pmer_from_str(optarg);
                break;
            case 'o': /* output file name */
                snprintf(a.fname, PATH_MAX - 1, optarg);
                break;
            default:
                exit(EXIT_FAILURE);
        }
    }
    return a;
}
