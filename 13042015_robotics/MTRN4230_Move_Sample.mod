MODULE MTRN4230_Move_Sample  
    
    VAR pose frameTable;
    VAR pose frameConv;
    VAR pose frameGlobal;
    PERS string share := "500050005000500050015000030005\\n";
    PERS bool startsync;
    
    CONST jointtarget Joint:=[[90,0,0,0,0,90],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ! Gobal frame
    
    CONST robtarget globalOrigin:=[[0,0,0],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    CONST robtarget global2:=[[30,0,0],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
    
    CONST robtarget global3:=[[30,30,0],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
    
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    ! conveyor home frame points
    
    !CONST robtarget pConvHome:=[[0,409,22],[0,-0.7071068,0.7071068,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    CONST robtarget pConv2 := [[30,409,22],[0,-0.7071068,0.7071068,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
 
    CONST robtarget pConv3 := [[30,439,22],[0,-0.7071068,0.7071068,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    ! table home
    !CONST robtarget pTableHome:=[[175,0,147],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    CONST robtarget pTable2:=[[200,0,147],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
 
    CONST robtarget pTable3:=[[200,30,147],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    CONST robtarget origin1:=[[0,0,0],[0,-0.7071068,0.7071068,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
    
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 
 
    
    ! The Main procedure. When you select 'PP to Main' on the FlexPendant, it will go to this procedure.
    PROC MainMove()
        !VAR pos pos1;
        VAR num loopCon := 1;
        VAR speeddata setSpeed;
        VAR jointtarget joints;
        VAR num axisNum;
        VAR bool axisOK;
        VAR string testAxis;
        VAR num ready := 1;
        VAR num check := 1;  
        VAR string checkStr;
        
        WaitUntil startsync;
        !frame definition
        frameGlobal:= DefFrame (globalOrigin, global2, global3);
        frameTable:= DefFrame (pTableHome, pTable2, pTable3);
        frameConv:= DefFrame (pConvHome, pConv2, pConv3);
        PDispSet frameGlobal; !activate the global frame
        
        share := "000000000000000000000000000000";

        MoveToCalibPos;
               
        linearMoveTo 300,0,400, v300;


        WHILE loopCon = 1 DO
            TPWrite share;
            read(share);  
            !TPWrite share;
       
        ENDWHILE
        
       
        
        
        ! Call another procedure, but provide some input arguments.
        !VariableSample pTableHome, 100, 100, 0, v100, fine;
        !pos1 := CPos(\Tool := tSCup \WObj := wobj0);
        !VariableSample CRobT(\Tool := tSCup \WObj := wobj0), 100, 100, 300, v100, fine;
        
    ENDPROC
    
    PROC MoveJSample()
    
        ! 'MoveJ' executes a joint motion towards a robtarget. This is used to move the robot quickly from one point to another when that 
        !   movement does not need to be in a straight line.
        ! 'pTableHome' is a robtarget defined in system module. The exact location of this on the table has been provided to you.
        ! 'v100' is a speeddata variable, and defines how fast the robot should move. The numbers is the speed in mm/sec, in this case 100mm/sec.
        ! 'fine' is a zonedata variable, and defines how close the robot should move to a point before executing its next command. 
        !   'fine' means very close, other values such as 'z10' or 'z50', will move within 10mm and 50mm respectively before executing the next command.
        ! 'tSCup' is a tooldata variable. This has been defined in a system module, and represents the tip of the suction cup, telling the robot that we
        !   want to move this point to the specified robtarget. Please be careful about what tool you use, as using the incorrect tool will result in
        !   the robot not moving where you would expect it to. Generally you should be using
        MoveJ pTableHome, v200, fine, tSCup;
        
    ENDPROC
    
    PROC MoveLSample()
        
        ! 'MoveL' will move in a straight line between 2 points. This should be used as you approach to pick up a chocolate
        ! 'Offs' is a function that is used to offset an existing robtarget by a specified x, y, and z. Here it will be offset 100mm in the positive z direction.
        !   Note that function are called using brackets, whilst procedures and called without brackets.
        MoveL Offs(pTableHome, 0, 0, 100), v100, fine, tSCup;
        
    ENDPROC
    
    PROC VariableSample(robtarget target, num x_offset, num y_offset, num z_offset, speeddata speed, zonedata zone)
        
        ! Call 'MoveL' with the input arguments provided.
        MoveJ Offs(target, x_offset, y_offset, z_offset), speed, zone, tSCup;
        
    ENDPROC

    
    PROC linearMove(num x_offset, num y_offset, num z_offset, speeddata speed)
        
        ! Call 'MoveL' with the input arguments provided.
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), x_offset, y_offset, z_offset), speed, fine, tSCup;
        
    ENDPROC    
 
    
    PROC jointMove(num ax1, num ax2, num ax3, num ax4, num ax5, num ax6, speeddata speed)
                
        VAR jointtarget current; 
        current := CJointT();
        
        current.robax.rax_1 := current.robax.rax_1 + ax1;
        current.robax.rax_2 := current.robax.rax_2 + ax2;
        current.robax.rax_3 := current.robax.rax_3 + ax3;
        current.robax.rax_4 := current.robax.rax_4 + ax4;
        current.robax.rax_5 := current.robax.rax_5 + ax5;
        current.robax.rax_6 := current.robax.rax_6 + ax6;
        
        MoveAbsJ current,speed,fine,tool0;
        
    ENDPROC 
        
    
    PROC linearMoveTo(num x, num y, num z, speeddata speed)
        
        VAR robtarget toWhere;
        toWhere:=[[x,y,z],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        MoveJ toWhere, v200, fine, tSCup;
        
    ENDPROC   
   
    PROC jointMoveTo(num ax1, num ax2, num ax3, num ax4, num ax5, num ax6, speeddata speed)
        
        VAR jointtarget current; 
        current := CJointT();
        
        current.robax.rax_1 := ax1;
        current.robax.rax_2 := ax2;
        current.robax.rax_3 := ax3;
        current.robax.rax_4 := ax4;
        current.robax.rax_5 := ax5;
        current.robax.rax_6 := ax6;       

        MoveAbsJ current,speed,fine,tool0;
        
    ENDPROC   
    
    
    
    
    ! linear movement along x, y, z axis
    
    PROC XLinearP(speeddata speed)
        
        ! x axis linear positive 
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), 5, 0, 0), speed, fine, tSCup;
    
    ENDPROC
  
    
    PROC XLinearN(speeddata speed)
   
        ! x axis linear negative 
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), -5, 0, 0), speed, fine, tSCup;
    
    ENDPROC
   
    
    PROC YLinearP(speeddata speed)
        
        ! y axis linear positive 
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), 0, 5, 0), speed, fine, tSCup;
    
    ENDPROC
  
    
    PROC YLinearN(speeddata speed)
        
        ! y axis linear negative
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), 0, -5, 0), speed, fine, tSCup;
    
    ENDPROC
    
    
    PROC ZLinearP(speeddata speed)
        
        ! z axis linear positive 
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), 0, 0, 5), speed, fine, tSCup;
    
    ENDPROC
  
    
    PROC ZLinearN(speeddata speed)
        
        ! z axis linear negative
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), 0, 0, -5), speed, fine, tSCup;
    
    ENDPROC 
    
    PROC MoveOnTable(num x_offset, num y_offset, speeddata speed)

        MoveJ Offs(origin1, x_offset, y_offset, 100), speed, fine, tSCup;       
        
    ENDPROC
    
    
    ! robot joint jogging 
    
    PROC axisC(num axis, speeddata speed)
        
        ! clockwise jogging
        VAR jointtarget current; 
        current := CJointT();
        
        
        TEST axis 
        
        CASE 1:
            current.robax.rax_1 := current.robax.rax_1 +1;                  
        CASE 2:
            current.robax.rax_2 := current.robax.rax_2 +1;
        CASE 3:
            current.robax.rax_3 := current.robax.rax_3 +1;
        CASE 4:
            current.robax.rax_4 := current.robax.rax_4 +1;
        CASE 5:
            current.robax.rax_5 := current.robax.rax_5 +1;
        CASE 6:
            current.robax.rax_6 := current.robax.rax_6 +1;
        DEFAULT:
            EXIT;
        ENDTEST
        
        MoveAbsJ current,speed,fine,tool0;
 
       
    ENDPROC  
    
    PROC axisAC(num axis, speeddata speed)
        
        ! Anti-clockwise jogging
        VAR jointtarget current; 
        current := CJointT();
        
        
        TEST axis 
        
        CASE 1:
            current.robax.rax_1 := current.robax.rax_1 -1;                  
        CASE 2:
            current.robax.rax_2 := current.robax.rax_2 -1;
        CASE 3:
            current.robax.rax_3 := current.robax.rax_3 -1;
        CASE 4:
            current.robax.rax_4 := current.robax.rax_4 -1;
        CASE 5:
            current.robax.rax_5 := current.robax.rax_5 -1;
        CASE 6:
            current.robax.rax_6 := current.robax.rax_6 -1;
        DEFAULT:
            EXIT;
        ENDTEST
        
        MoveAbsJ current,speed,fine,tool0;
 
       
    ENDPROC
    
    ! active whatever frame    global frame/base frame: default, table frame: 1, conveyor frame: 2 
    PROC activeFrame(num frame)
        PDispOff;
        
        TEST frame
        
        CASE 1: PDispSet frameTable;
        
        CASE 2: PDispSet frameConv;
        
        DEFAULT: PDispSet frameGlobal;
        
        ENDTEST
                    
    ENDPROC
 
    
    
    ! Check rechability
    FUNC bool IsReachable(num x, num y, num z, PERS tooldata ToolReach, PERS wobjdata WobjReach)
        
        VAR robtarget target;
        VAR bool bReachable;
        VAR jointtarget jntReach;
        
        target:=[[x,y,z],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        bReachable := TRUE;
  
        jntReach := CalcJointT(target, ToolReach\Wobj:=WobjReach);
  
        RETURN bReachable;
  
        ERROR
        
        IF ERRNO = ERR_ROBLIMIT THEN
            bReachable := FALSE;
            TRYNEXT;
        ENDIF
        
    ENDFUNC      
  
    
    ! read data 
    PROC read(string data)
        ! check command 
        VAR string commandStr;
        VAR num commandNum;
        VAR string speedStr;
        VAR num speedNum;
        VAR speeddata speed;

        VAR string data1Str;
        VAR string data2Str;
        VAR string data3Str;
        VAR string data4Str;
        VAR string data5Str;
        VAR string data6Str;        
       
        VAR num data1Num;
        VAR num data2Num;
        VAR num data3Num;
        VAR num data4Num;
        VAR num data5Num;
        VAR num data6Num;
        
        VAR num dataNum;
        
        VAR bool ok;
        
        commandStr:= StrPart(data,29,2);
        ok := StrToVal(commandStr,commandNum);
        
        speedStr:= StrPart(data,25,4);
        ok := StrToVal(speedStr,speedNum);
        speed := [speedNum, speedNum, speedNum, speedNum];
        
        data1Str:= StrPart(data,21,4);
        data2Str:= StrPart(data,17,4);
        data3Str:= StrPart(data,13,4);
        data4Str:= StrPart(data,9,4);
        data5Str:= StrPart(data,5,4);
        data6Str:= StrPart(data,1,4);
        
        ok := StrToVal(data1Str,data1Num);
        ok := StrToVal(data2Str,data2Num);
        ok := StrToVal(data3Str,data3Num);
        ok := StrToVal(data4Str,data4Num);
        ok := StrToVal(data5Str,data5Num);
        ok := StrToVal(data6Str,data6Num);
        
        data1Num := data1Num - 5000;
        data2Num := data2Num - 5000;
        data3Num := data3Num - 5000;
        data4Num := data4Num - 5000;
        data5Num := data5Num - 5000;
        data6Num := data6Num - 5000;
        
        TEST commandNum 
        CASE 0:
            TPWrite "Nothing need to do";
            WaitTime 0.5;
        CASE 1:
            TPWrite " got the linear Move";
            linearMove data1Num, data2Num, data3Num, speed; 
        CASE 2:
            TPWrite " got the joint Move";
            jointMove data1Num, data2Num, data3Num, data4Num, data5Num, data6Num, speed;            
        CASE 3:
            TPWrite " got the linear Move To";
            linearMoveTo data1Num, data2Num, data3Num, speed; 
        CASE 4:
            TPWrite " got the joint Move To";
            jointMoveTo data1Num, data2Num, data3Num, data4Num, data5Num, data6Num, speed;  
        CASE 5:
            TPWrite " linear jogging";
            IF data1Num > 0.9 THEN
                XLinearP speed;
            ELSEIF data1Num < -0.9 THEN
                XLinearN speed;
            ELSEIF data2Num > 0.9 THEN
                YLinearP speed;
            ELSEIF data2Num < -0.9 THEN
                YLinearN speed;
            ELSEIF data3Num > 0.9 THEN
                ZLinearP speed;
            ELSEIF data3Num < -0.9 THEN
                ZLinearN speed;
            ENDIF           
        CASE 6:
            TPWrite " move on table";
            MoveOnTable data1Num, data2Num, speed;
        CASE 7:
            TPWrite " joint jogging";
            IF data1Num > 0.9 THEN
                axisC 1, speed;
            ELSEIF data1Num < -0.9 THEN
                axisAC 1, speed;
            ELSEIF data2Num > 0.9 THEN
                axisC 2, speed;
            ELSEIF data2Num < -0.9 THEN
                axisAC 2, speed;  
            ELSEIF data3Num > 0.9 THEN
                axisC 3, speed;
            ELSEIF data3Num < -0.9 THEN
                axisAC 3, speed;
            ELSEIF data4Num > 0.9 THEN
                axisC 4, speed;
            ELSEIF data4Num < -0.9 THEN
                axisAC 4, speed;                
            ELSEIF data5Num > 0.9 THEN
                axisC 5, speed;
            ELSEIF data5Num < -0.9 THEN
                axisAC 5, speed; 
            ELSEIF data6Num > 0.9 THEN
                axisC 6, speed;
            ELSEIF data6Num < -0.9 THEN
                axisAC 6, speed;                                
            ENDIF
            
        CASE 8:
            TPWrite " activate frame";
            activeFrame speedNum;    ! on the speedNum 4 bits in the share data       
        DEFAULT:
            EXIT;
        ENDTEST
         
        
        
    ENDPROC   
       
    
    
    
ENDMODULE