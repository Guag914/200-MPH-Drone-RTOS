.syntax unified
.cpu cortex-m7
.thumb

.global PendSV_Handler
.thumb_func
.type PendSV_Handler, %function

.extern currTaskAddress
.extern nextTaskAddress

PendSV_Handler:
    CPSID I
    MRS R0, PSP
    STMDB R0!, {R4-R11}

    LDR R1, =currTaskAddress
    LDR R1, [R1]
    STR R0, [R1]

    LDR R1, =nextTaskAddress
    LDR R1, [R1]
    LDR R0, [R1]

    LDMIA R0!, {R4-R11}
    MSR PSP, R0
    CPSIE I
    BX LR