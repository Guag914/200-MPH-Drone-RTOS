.syntax unified
.cpu cortex-m7
.thumb

.global PendSV_Handler
.thumb_func
.type PendSV_Handler, %function

.extern nextExcReturn

PendSV_Handler:
    CPSID I
    MRS R0, PSP
    VSTMDB R0!, {S16-S31}
    STMDB R0!, {R4-R11}

    MOV R1, LR
    BL  decideNextTask

    LDR R1, =nextExcReturn
    LDR R1, [R1]
    MOV LR, R1

    LDMIA R0!, {R4-R11}
    VLDMIA R0!, {S16-S31}
    MSR PSP, R0

    CPSIE I
    BX LR