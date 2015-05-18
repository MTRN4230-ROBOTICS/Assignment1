MODULE MTRN4230_IO_Sample
    
        
   ! author: Ziwei Guo, Zicong He 
   ! late update: 20/04/2015
   !
   
    PROC MainIO()
        
        TurnVacOn;
        
        ! Time to wait in seconds.
        WaitTime 2;
        
        TurnVacOff;
        
        WaitTime 2;
        
        TurnVacSoleniodOn;
        
        WaitTime 2;
        
        TurnVacSoleniodOff;
        
        WaitTime 2;
        
        TurnConOn;
        
        WaitTime 2;
        
        ConveyorForward;
        
        WaitTime 2;
        
        ConveyorBackward;
        
        WaitTime 2;
        
        TurnConOff;
        
    ENDPROC
    
    !Turn on vacumn
    PROC TurnVacOn()
        
        ! Set VacRun on.
        SetDO DO10_1, 1;
        
    ENDPROC
    
    !Turn off vacumn
    PROC TurnVacOff()
        
        ! Set VacRun off.
        SetDO DO10_1, 0;
        
    ENDPROC
  
    !Start to suck
    PROC TurnVacSoleniodOn()
        
        ! Set VacRun soleniod on.
        SetDO DO10_2, 1;
        
    ENDPROC
    
    !Stop sucking
    PROC TurnVacSoleniodOff()
        
        ! Set VacRun on.
        SetDO DO10_2, 0;
        
    ENDPROC   
    
    !Turn on conveyor
    PROC TurnConOn()
        SetDO DO10_3, 1;
    ENDPROC
    
    !Turn off conveyor
    PROC TurnConOff()
        SetDO DO10_3, 0;
    ENDPROC
   
    !Set conveyor direction into 1
    PROC ConveyorForward()
        SetDO DO10_4, 1;
    ENDPROC
    
    !Set conveyor direction into 0
     PROC ConveyorBackward()
        SetDO DO10_4, 0;
    ENDPROC   
 
    
ENDMODULE