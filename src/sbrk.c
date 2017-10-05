/***************************************************************/
/*                                                             */
/*      PROJECT NAME :  BasicMouse1                            */
/*      FILE         :  sbrk.c                                 */
/*      DESCRIPTION  :  Program of sbrk                        */
/*      CPU SERIES   :  SH-2                                   */
/*      CPU TYPE     :  SH7125                                 */
/*                                                             */
/*      This file is generated by e2 studio.                   */
/*                                                             */
/***************************************************************/                                 


#include <stddef.h>
#include <stdio.h>
#include "typedefine.h"
#include "sbrk.h"

_SBYTE  *sbrk(size_t size);

//const size_t _sbrk_size=        /* Specifies the minimum unit of    */
                    /* the defined heap area        */

extern _SBYTE *_s1ptr;

#pragma pack 4
union HEAP_TYPE {
    _SDWORD  dummy ;        /* Dummy for 4-byte boundary            */
    _SBYTE heap[HEAPSIZE];    /* Declaration of the area managed by sbrk    */
};

static union HEAP_TYPE heap_area ;
//static __X union HEAP_TYPE heap_area__X;              /* for DSP-C */
//static __Y union HEAP_TYPE heap_area__Y;              /* for DSP-C */
#pragma unpack

/* End address allocated by sbrk    */
static _SBYTE *brk=(_SBYTE *)&heap_area;
//static __X _SBYTE *brk__X=(_SBYTE __X *)&heap_area__X;    /* for DSP-C */
//static __Y _SBYTE *brk__Y=(_SBYTE __Y *)&heap_area__Y;    /* for DSP-C */

/**************************************************************************/
/*     sbrk:Memory area allocation                                        */
/*          Return value:Start address of allocated area (Pass)           */
/*                       -1                              (Failure)        */
/**************************************************************************/
_SBYTE  *sbrk(size_t size)                      /* Assigned area size   */
{
    _SBYTE  *p;

    if(brk+size > heap_area.heap+HEAPSIZE){     /* Empty area size      */
        p = (_SBYTE *)-1;
    }
    else {
        p = brk;                                /* Area assignment      */
        brk += size;                            /* End address update   */
    }
    return p;
}

/**************************************************************************/
/*     sbrk:X Memory area allocation                                      */
/*          Return value:Start address of allocated area (Pass)           */
/*                       -1                              (Failure)        */
//*      When the dspc option is specified at compiling, remove // of     */
//*      the head of the line which has /* for DSP-C */ and add start     */
//*      options and add "$XB" and "$YB" to the start option at linkage.  */
/**************************************************************************/
//_SBYTE __X *sbrk__X(size_t size)        /* Assigned area size */      /* for DSP-C */
//{                                                                     /* for DSP-C */
//    __X _SBYTE *p;                                                    /* for DSP-C */
//                                                                      /* for DSP-C */
//    if (brk__X+size > heap_area__X.heap+HEAPSIZE) { /* Empty area size */ /* for DSP-C */
//        return (_SBYTE __X *)-1;                                      /* for DSP-C */
//    }                                                                 /* for DSP-C */
//                                                                      /* for DSP-C */
//    p = brk__X;                 /* Area assignment */                 /* for DSP-C */
//    brk__X += size;             /* End address update */              /* for DSP-C */
//    return p;                                                         /* for DSP-C */
//}                                                                     /* for DSP-C */

/**************************************************************************/
/*     sbrk:Y Memory area allocation                                      */
/*          Return value:Start address of allocated area (Pass)           */
/*                       -1                              (Failure)        */
//*      When the dspc option is specified at compiling, remove // of     */
//*      the head of the line which has /* for DSP-C */ and add start     */
//*      options and add "$XB" and "$YB" to the start option at linkage.  */
/**************************************************************************/
//_SBYTE __Y *sbrk__Y(size_t size)         /* Assigned area size */     /* for DSP-C */
//{                                                                     /* for DSP-C */
//    __Y _SBYTE *p;                                                    /* for DSP-C */
//                                                                      /* for DSP-C */
//    if (brk__Y+size > heap_area__Y.heap+HEAPSIZE) { /* Empty area size */ /* for DSP-C */
//        return (_SBYTE __Y *)-1;                                      /* for DSP-C */
//    }                                                                 /* for DSP-C */
//    p = brk__Y;                  /* Area assignment */                /* for DSP-C */
//    brk__Y += size;              /* End address update */             /* for DSP-C */
//    return p;                                                         /* for DSP-C */
//}                                                                     /* for DSP-C */
