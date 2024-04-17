    ;SECTION .text:CODE
    AREA ||.text||, CODE, READONLY

        ;AAPCS INTERWORK, ROPI, RWPI_COMPATIBLE,  VFP_COMPATIBLE
        PRESERVE8

    ;/* Define public symbols.  */

    EXPORT __txm_module_preamble


    ;/* Define application-specific start/stop entry points for the module.  */

    EXTERN appMain


    ;/* Define common external refrences.  */

        IMPORT  _txm_module_thread_shell_entry
        IMPORT  _txm_module_callback_request_thread_entry
        IMPORT  |Image$$ER_RO$$Length|
        IMPORT  |Image$$ER_RW$$Length|
		IMPORT  |Image$$ER_RW$$RW$$Length|
		IMPORT  |Image$$ER_RW$$ZI$$Length|
		IMPORT  |Image$$ER_ZI$$ZI$$Length|

       ; DATA
__txm_module_preamble
        DCD      0x4D4F4455                                        ; Module ID
        DCD      0x5                                               ; Module Major Version
        DCD      0x6                                               ; Module Minor Version
        DCD      32                                                ; Module Preamble Size in 32-bit words
        DCD      0x12345678                                        ; Module ID (application defined) 
        DCD      0x02000007                                        ; Module Properties where:
                                                                    ;   Bits 31-24: Compiler ID
                                                                    ;           0 -> IAR
                                                                    ;           1 -> RVDS
                                                                    ;           2 -> GNU
                                                                    ;   Bit 0:  0 -> Privileged mode execution
                                                                    ;           1 -> User mode execution
                                                                    ;   Bit 1:  0 -> No MPU protection
                                                                    ;           1 -> MPU protection (must have user mode selected)
                                                                    ;   Bit 2:  0 -> Disable shared/external memory access
                                                                    ;           1 -> Enable shared/external memory access
        DCD      _txm_module_thread_shell_entry - . - 0             ; Module Shell Entry Point
        DCD      appMain - . - 0                                    ; Module Start Thread Entry Point
        DCD      0                                                  ; Module Stop Thread Entry Point 
        DCD      21                                                  ; Module Start/Stop Thread Priority
        DCD      8192                                               ; Module Start/Stop Thread Stack Size
        DCD      _txm_module_callback_request_thread_entry - . - 0  ; Module Callback Thread Entry
        DCD      19                                                  ; Module Callback Thread Priority     
        DCD      4096                                               ; Module Callback Thread Stack Size    
        DCD      |Image$$ER_RO$$Length| + |Image$$ER_RW$$Length|    ; Module Code Size + RW Load region size
        DCD      |Image$$ER_RW$$Length| + |Image$$ER_ZI$$ZI$$Length|; Module Data Size
        DCD      0                                                  ; Reserved 0   
        DCD      0                                                  ; Reserved 1
        DCD      0                                                  ; Reserved 2
        DCD      0                                                  ; Reserved 3
        DCD      0                                                  ; Reserved 4
        DCD      0                                                  ; Reserved 5     
        DCD      0                                                  ; Reserved 6     
        DCD      0                                                  ; Reserved 7   
        DCD      0                                                  ; Reserved 8  
        DCD      0                                                  ; Reserved 9
        DCD      0                                                  ; Reserved 10
        DCD      0                                                  ; Reserved 11
        DCD      0                                                  ; Reserved 12
        DCD      0                                                  ; Reserved 13
        DCD      0                                                  ; Reserved 14
        DCD      0                                                  ; Reserved 15

        END


