MODULE MTRN4230_Server_Sample    
    
   ! author: Ziwei Guo, Zicong He 
   ! late update: 20/04/2015
   !
   
    ! The socket connected to the client.
    VAR socketdev client_socket;
    
    ! The host and port that we will be listening for a connection on.
    CONST string host := "192.168.2.1";
    !CONST string host := "127.0.0.1";
    CONST num port := 7027;
    PERS string share;
    VAR string received_str;
    VAR string feedback;
    VAR num refresh:=1;
    
    VAR num retrycount := 0; 
    VAR string retrycountStr;
    
    PERS num timeCount := 99;
    
    VAR num try:=1;
    VAR string tryStr;
    
    VAR string lastCommand;
    VAR string storeLast;
    
    ! define work object and tool
    PERS wobjdata wTable;
    PERS wobjdata wConv;
    PERS tooldata tSCup;
    PERS bool startsync := TRUE;
   
    PROC main() 
            startsync := TRUE;
            
            IF refresh = 1 THEN 
                ListenForAndAcceptConnection;
                refresh := 0 ;   
                SocketSend client_socket \Str:=("000000000000000000000000000099\0A");
                checkRetry;
            ENDIF
           
            !TPWrite received_str;
            !TPWrite "Still trying to connect";
            ! Receive a string from the client.
            SocketReceive client_socket \Str:=received_str, \Time:= 60;
            !,\Time:=WAIT_MAX;
            
            !-----
            IF checkStatus(received_str) < 90 THEN
                ! check pause
               IF checkStatus(received_str) = 9 THEN
                   
                   lastCommand := storeLast;        
                   StopMove; 
                   
               ENDIF
               
               ! check restart
               IF checkStatus(received_str) = 10 OR checkStatus(received_str) = 20 THEN
                   StartMove;
                   share := lastCommand;
               ENDIF
               
               share := received_str;
               feedback :=readPose() + "99";
               
               ! store last command
               IF checkStatus(received_str) < 9 OR checkStatus(received_str) > 10 THEN
                   storeLast:= share;
               ENDIF
               
               IF checkStatus(received_str) = 20 THEN
                   checkRetry;
               ENDIF
                             
                ! Send the string back to the client, adding a line feed character.
                SocketSend client_socket \Str:=(feedback+"\0A");
            ELSE     
                SocketSend client_socket \Str:=(received_str+"\0A");
            ENDIF
            ! check conveyor status
            !checkConStat;
            ERROR 
                SocketClose client_socket;
                ListenForAndAcceptConnection;
                SocketSend client_socket \Str:=("000000000000000000000000000099\0A");
                ResetRetryCount;
                retrycount := retrycount + 1;
                retrycountStr := NumToStr(retrycount,0);
                TPWrite retrycountStr; 
                RETRY;
            

                       
    ENDPROC

    
    !Listen and accept matlab connection 
    ! relative to a frame, input: x, y, z point and speed
    PROC ListenForAndAcceptConnection()
        
        ! Create the socket to listen for a connection on.
        VAR socketdev welcome_socket;
        SocketCreate welcome_socket;
        
        ! Bind the socket to the host and port.
        SocketBind welcome_socket, host, port;
        
        ! Listen on the welcome socket.
        SocketListen welcome_socket;
        
        ! Accept a connection on the host and port.
        SocketAccept welcome_socket, client_socket, \Time:=WAIT_MAX;
        !SocketAccept welcome_socket, client_socket;
        
        ! Close the welcome socket, as it is no longer needed.
        SocketClose welcome_socket;
        
        ERROR
            TPWrite "Succeed to retry";
            RETRY;
    ENDPROC
    
    ! Close the connection to the client.
    PROC CloseConnection()
        SocketClose client_socket;
    ENDPROC
    
    
    
   !check position and joint angles, return as string "joint + zyx"
   FUNC string readPose()
  
        VAR string currentPoseStrX;
        VAR string currentPoseStrY;
        VAR string currentPoseStrZ;
        VAR string currentPoseStr;
        VAR string currentJointStr1;
        VAR string currentJointStr2;
        VAR string currentJointStr3;
        VAR string currentJointStr4;
        VAR string currentJointStr5;
        VAR string currentJointStr6;
        VAR pos currentPosition;  ! only contain x,y,z
        VAR jointtarget currentAngle; 
        
                   
        currentPosition := CPos(\Tool:= tSCup);
        !\WObj:= wTable
        currentPoseStrX := NumToStr (currentPosition.x + 5000,0);
        currentPoseStrY := NumToStr (currentPosition.y + 5000,0);
        currentPoseStrZ := NumToStr (currentPosition.z + 5000,0);
        
        ! joint angle feedback
        currentAngle := CJointT();         
        currentJointStr1 := NumToStr(currentAngle.robax.rax_1/4 +50, 0) ;
        currentJointStr2 := NumToStr(currentAngle.robax.rax_2/4 +50, 0) ;
        currentJointStr3 := NumToStr(currentAngle.robax.rax_3/4 +50, 0) ;
        currentJointStr4 := NumToStr(currentAngle.robax.rax_4/4 +50, 0) ;
        currentJointStr5 := NumToStr(currentAngle.robax.rax_5/4 +50, 0) ;
        currentJointStr6 := NumToStr(currentAngle.robax.rax_6/4 +50, 0) ;
        !currentJointStr1 := NumToStr(currentAngle.robax.rax_1, 0) ;
        !TPWrite ("Axis1 is " + currentJointStr1);
        !current.robax.rax_1 
        !current.robax.rax_1 
        !current.robax.rax_1 
        
        
        ! make it up to the four bits format
        currentPoseStr :=  currentJointStr6 + currentJointStr5 + currentJointStr4 + currentJointStr3 + currentJointStr2 + currentJointStr1 + currentPoseStrZ + currentPoseStrY + currentPoseStrX + "0000";
        RETURN currentPoseStr;
        
    ENDFUNC

    ! for returning 4 bits string (only for read pose function)
    ! Input: string with less than 4 bytes
    FUNC string fourBit(string value)
        VAR num bitNum;
        
        bitNum := StrLen(value);
        
       TEST bitNum 
        
        CASE 1:
            value := "000" + value;                 
        CASE 2:
            value := "00" + value;  
        CASE 3:
            value := "0" + value;  
            
        DEFAULT:
            EXIT;
        ENDTEST
        
        RETURN value;
        
    ENDFUNC
    
    !Return last 2 digits in the string in int format
    !Input: string value
    FUNC num checkStatus(string value)
        VAR num status;
        VAR bool ok;
        
        value := strPart(value,29,2);
        ok := StrToVal(value, status);
        
        RETURN status;
        
    ENDFUNC
  
    ! Set ConRun to 0 if ConStat is set to 0
    PROC checkConStat()
    
        IF DI10_1 = 0 THEN
            SetDO DO10_3, 0;
            TPWrite " ConRun is set to 0 ";
        ENDIF
        
    ENDPROC
    
    !Restart program
    !no input needed
    PROC checkRetry()
           
                   TPWrite "In the loop";
            ERROR
            IF ERRNO = ERR_SOCK_TIMEOUT THEN
                
                   TPWrite "Retry~~~~~~~~~~~~!!!!!!!!!!!!";
                 RETRY;
            ELSE
                TPWrite "Wrong data type";
            ENDIF
                 
    ENDPROC
 
ENDMODULE