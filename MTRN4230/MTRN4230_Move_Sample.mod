MODULE MTRN4230_Move_Sample 
    
   ! author: Ziwei Guo, Zicong He 
   ! late update: 20/04/2015
   !
   
   ! initialise variables 
    VAR pose frameTable;
    VAR pose frameConv;
    VAR pose frameGlobal;    
    VAR pose frameTableCam;
    VAR pose frameConveyorCam;
    PERS string share;
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
    ! table Camera frame 
    
    CONST robtarget tableCamOrigin:=[[400,0,1070],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    CONST robtarget tableCam2:=[[430,0,1070],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
    
    CONST robtarget tableCam3:=[[430,30,1070],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
  
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!     
    ! conveyor Camera frame 
    
    CONST robtarget conveyorCamOrigin:=[[0,375,860],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    CONST robtarget conveyorCam2:=[[30,375,860],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
    
    CONST robtarget conveyorCam3:=[[30,400,860],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
  
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
        
    
    ! The Main procedure. When you select 'PP to Main' on the FlexPendant, it will go to this procedure.
    PROC main()
        
        ! initialise values 
        VAR num loopCon := 1;
        VAR speeddata setSpeed;
        VAR jointtarget joints;
        VAR num axisNum;
        VAR bool axisOK;
        VAR string testAxis;
        VAR num ready := 1;
        VAR num check := 1;  
        VAR string checkStr;
        
        VAR string str1;
        VAR string str2;
        VAR string str3;
        VAR string str4;
        VAR string str5;
        VAR string str6;        
        VAR jointtarget current; 

        
        !wait for matlab to sync
        WaitUntil startsync;
        
        !frame definition
        frameGlobal:= DefFrame (globalOrigin, global2, global3);
        frameTable:= DefFrame (pTableHome, pTable2, pTable3);
        frameConv:= DefFrame (pConvHome, pConv2, pConv3);
        frameTableCam:= DefFrame (tableCamOrigin, tableCam2, tableCam3);
        frameConveyorCam:= DefFrame (conveyorCamOrigin, conveyorCam2, conveyorCam3);
        
        !activate default frame: the global frame
        PDispSet frameGlobal; 
        
        !initial the receive values to avoid errors
        share := "000000000000000000000000000000";

        !Home position
        MoveToCalibPos;
        
        ! To aviod sigularity
        
        !linearMoveTo 0,0,500,v300;
     
        ! TEST SECTION
        !activeFrame 4;        
        !linearMoveTo 0,0,-500,v100;        
        !linearMoveTo -100,200,400, v50;
      
        !ERROR
         !       IF ERRNO = ERR_SGUN_ESTOP THEN
         !           TPWrite "got the error";
                !StorePath;
                !p_err := CRobT(\Tool:= tool1 \WObj:=wobj0);
                !! Fix the problem
                !MoveL p_err, v100, fine, tool1;
                !RestoPath;
                !StartMoveRetry;
          !      ENDIF
            
        ! Keep Reeding data from matlab
        WHILE loopCon = 1 DO
     
            !TPWrite share;
            WaitTime 0.1;           
            read(share);  
                   
        ENDWHILE
                       
        
        ! Call another procedure, but provide some input arguments.
        !VariableSample pTableHome, 100, 100, 0, v100, fine;
        !pos1 := CPos(\Tool := tSCup \WObj := wobj0);
        !VariableSample CRobT(\Tool := tSCup \WObj := wobj0), 100, 100, 300, v100, fine;
        
    ENDPROC
  
    ! move the end effector PtableHome 
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
    
    ! linear Move by offset from the table by 10 cm
    PROC MoveLSample()
        
        ! 'MoveL' will move in a straight line between 2 points. This should be used as you approach to pick up a chocolate
        ! 'Offs' is a function that is used to offset an existing robtarget by a specified x, y, and z. Here it will be offset 100mm in the positive z direction.
        !   Note that function are called using brackets, whilst procedures and called without brackets.
        MoveL Offs(pTableHome, 0, 0, 100), v100, fine, tSCup;
        
    ENDPROC
    
    ! joint linear move
    ! input: reference point, x offset, y offset, z offset, speed, zone)
    PROC VariableSample(robtarget target, num x_offset, num y_offset, num z_offset, speeddata speed, zonedata zone)
        
        ! Call 'MoveL' with the input arguments provided.
        MoveJ Offs(target, x_offset, y_offset, z_offset), speed, zone, tSCup;
        
    ENDPROC

    
    ! linear mode move(jogging)
    ! input: x, y, z offset and speed
    PROC linearMove(num x_offset, num y_offset, num z_offset, speeddata speed)
        
        ! Call 'MoveL' with the input arguments provided.
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), x_offset, y_offset, z_offset), speed, fine, tSCup;
        
    ENDPROC    
 
    
    ! rotational mode (jogging)
    ! input which axis to control, speed
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
        
    !linear move to a point 
    ! relative to a frame, input: x, y, z point and speed
    PROC linearMoveTo(num x, num y, num z, speeddata speed)
        
        VAR robtarget toWhere;
        toWhere:=[[x,y,z],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        MoveJ toWhere, speed, fine, tSCup;
        
    ENDPROC   
   
    ! controls to each joint to a specific angle 
    ! input: input angles to each axis and speed
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
    ! linear move along x, positive direction, input speed 
    PROC XLinearP(speeddata speed)
        
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), 5, 0, 0), speed, fine, tSCup;
    
    ENDPROC
  
    ! linear move along x, negative direction, input speed 
    PROC XLinearN(speeddata speed)
   
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), -5, 0, 0), speed, fine, tSCup;
    
    ENDPROC
   
    ! linear move along y, positive direction, input speed 
    PROC YLinearP(speeddata speed)
        
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), 0, 5, 0), speed, fine, tSCup;
    
    ENDPROC
  
    ! linear move along y, negative direction, input speed  
    PROC YLinearN(speeddata speed)
        
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), 0, -5, 0), speed, fine, tSCup;
    
    ENDPROC
    
     ! linear move along z, positive direction, input speed 
    PROC ZLinearP(speeddata speed)
        
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), 0, 0, 5), speed, fine, tSCup;
    
    ENDPROC
  
     ! linear move along z, negative direction, input speed 
    PROC ZLinearN(speeddata speed)
        
        MoveL Offs(CRobT(\Tool := tSCup \WObj := wobj0), 0, 0, -5), speed, fine, tSCup;
    
    ENDPROC 
    
    ! get points from camera, planner move on the table in 10cm height
    ! reference to PtableHome as origin. input: x, y offset, speed
    PROC MoveOnTable(num x_offset, num y_offset, speeddata speed)

        MoveJ Offs(origin1, x_offset, y_offset, 100), speed, fine, tSCup;       
        
    ENDPROC
    
    
    ! robot joint jogging
    
    ! clockwise jogging
    ! specify the which axis, speed
    PROC axisC(num axis, speeddata speed)
                
        VAR jointtarget current; 
        current := CJointT();
               
        TEST axis 
        
        CASE 1:
            current.robax.rax_1 := current.robax.rax_1 +4;                  
        CASE 2:
            current.robax.rax_2 := current.robax.rax_2 +4;
        CASE 3:
            current.robax.rax_3 := current.robax.rax_3 +4;
        CASE 4:
            current.robax.rax_4 := current.robax.rax_4 +4;
        CASE 5:
            current.robax.rax_5 := current.robax.rax_5 +4;
        CASE 6:
            current.robax.rax_6 := current.robax.rax_6 +4;
        DEFAULT:
            EXIT;
        ENDTEST
        
        MoveAbsJ current,speed,fine,tool0;
 
       
    ENDPROC  

    ! Anti-clockwise jogging
    ! specify the which axis, speed
    PROC axisAC(num axis, speeddata speed)
        
        VAR jointtarget current; 
        current := CJointT();
                
        TEST axis 
        
        CASE 1:
            current.robax.rax_1 := current.robax.rax_1 -4;                  
        CASE 2:
            current.robax.rax_2 := current.robax.rax_2 -4;
        CASE 3:
            current.robax.rax_3 := current.robax.rax_3 -4;
        CASE 4:
            current.robax.rax_4 := current.robax.rax_4 -4;
        CASE 5:
            current.robax.rax_5 := current.robax.rax_5 -4;
        CASE 6:
            current.robax.rax_6 := current.robax.rax_6 -4;
        DEFAULT:
            EXIT;
        ENDTEST
        
        MoveAbsJ current,speed,fine,tool0;
        
    ENDPROC
    
    ! active whatever frame    global frame/base frame: default, table frame: 1, conveyor frame: 2
    ! table camera: 3,  converyor camera: 4 
    PROC activeFrame(num frame)
        
        PDispOff;
        
        TEST frame
        
        CASE 1: PDispSet frameTable;
        
        CASE 2: PDispSet frameConv;
        
        CASE 3: PDispSet frameTableCam;
        
        CASE 4: PDispSet frameConveyorCam;
        
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

    ! read data from matlab
    ! which function is asked to do. input recived data     
    PROC read(string data)
        
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
        
        ! data is total 30 bits
        ! Command code: get 29th and 30th bits
        commandStr:= StrPart(data,29,2);
        ok := StrToVal(commandStr,commandNum);
        
        ! Speed data: get 25th and 28th bits
        speedStr:= StrPart(data,25,4);
        ok := StrToVal(speedStr,speedNum);
        speed := [speedNum, speedNum, speedNum, speedNum];
        
        ! obtain data bits:
        ! data1(21~24 bits): x position/ axis 1 angle (degree)/ axis 1 actication(0: off, 1: clockwise -1: anti-clockwise)
        ! data2(17~20 bits): y position/ axis 2 angle (degree)/ axis 1 actication(0: off, 1: clockwise -1: anti-clockwise)
        ! data3(13~16 bits): z position/ axis 3 angle (degree)/ axis 1 actication(0: off, 1: clockwise -1: anti-clockwise)
        ! data4(9~12 bits): axis 4 angle (degree)/ axis 4 actication(0: off, 1: clockwise -1: anti-clockwise)
        ! data5(5~8 bits):  axis 5 angle (degree)/ axis 5 actication(0: off, 1: clockwise -1: anti-clockwise)
        ! data6(1~4 bits):  axis 6 angle (degree)/ axis 6 actication(0: off, 1: clockwise -1: anti-clockwise)
        
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
        
        ! 5000 is used to avoid of transfering minus side in the share string
        ! 5000 correction 
        data1Num := data1Num - 5000;
        data2Num := data2Num - 5000;
        data3Num := data3Num - 5000;
        data4Num := data4Num - 5000;
        data5Num := data5Num - 5000;
        data6Num := data6Num - 5000;
        
        ! check function to be used 
        TEST commandNum 
        CASE 0:
            !TPWrite "Nothing need to do";
            !WaitTime 0.5;
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
            
        CASE 9:
            TPWrite " now pause, waiting to move";
            
        CASE 10:
            TPWrite " robot resume";
 
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            ! IO input use the speed data bit to contain on/off signal from matlab
        CASE 11:
        
            IF speedNum > 0.9 THEN
                TPWrite " pump on ";
                TurnVacOn;
            ELSE
                TPWrite " pump off ";
                TurnVacOff;
            ENDIF
                       
        CASE 12:
        
            IF speedNum > 0.9 THEN
                TPWrite " soleniod on ";
                TurnVacSoleniodOn;
            ELSE
                TPWrite " soleniod off";
                TurnVacSoleniodOff;
            ENDIF
            
        CASE 13:
        
            IF speedNum > 0.9 THEN
                TPWrite " conveyor on ";
                TurnConOn;
            ELSE
                TPWrite " conveyor off ";
                TurnConOff;                
            ENDIF
                    
        CASE 14:
                 
            IF speedNum > 0.9 THEN
                TPWrite " direction forward ";
                ConveyorForward;
            ELSE
                TPWrite " direction backward ";
                ConveyorBackward;                
            ENDIF            

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!            
        DEFAULT:
            EXIT;
        ENDTEST
               
        
    ENDPROC   
            
    
ENDMODULE