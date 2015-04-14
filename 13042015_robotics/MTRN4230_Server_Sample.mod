MODULE MTRN4230_Server_Sample    
	
    ! The socket connected to the client.
    VAR socketdev client_socket;
    
    ! The host and port that we will be listening for a connection on.
    !CONST string host := "192.168.2.1";
    CONST string host := "127.0.0.1";
    CONST num port := 7027;
    PERS string share;
    VAR string received_str;
    VAR string feedback;
    VAR num refresh:=1;
    
    VAR num try:=1;
    VAR string tryStr;
    
    
    ! define work object and tool
    PERS wobjdata wTable;
    PERS wobjdata wConv;
    PERS tooldata tSCup;
    PERS bool startsync := TRUE;
   
    PROC MainServer() 
            startsync := TRUE;
            
            IF refresh = 1 THEN 
                ListenForAndAcceptConnection;
                refresh := 0 ;                
                SocketSend client_socket \Str:=("000000000000000000000000000099\0A");
            ENDIF
            
            TPWrite received_str;
            TPWrite "got";
            ! Receive a string from the client.
            SocketReceive client_socket \Str:=received_str;
            
            !-----
            IF checkStatus(received_str) < 90 THEN
                
                share := received_str;
               feedback :=readPose() + "99";
                ! Send the string back to the client, adding a line feed character.
                SocketSend client_socket \Str:=(feedback+"\0A");
            ELSE     
                SocketSend client_socket \Str:=(received_str+"\0A");
            ENDIF
            ! check conveyor status
            checkConStat;
            
            !try:= try +1;
            !tryStr := NumToStr(try,0);
            !SocketSend client_socket \Str:=(tryStr + "\0A");
            
            !CloseConnection;           
            
    ENDPROC

    PROC ListenForAndAcceptConnection()
        
        ! Create the socket to listen for a connection on.
        VAR socketdev welcome_socket;
        SocketCreate welcome_socket;
        
        ! Bind the socket to the host and port.
        SocketBind welcome_socket, host, port;
        
        ! Listen on the welcome socket.
        SocketListen welcome_socket;
        
        ! Accept a connection on the host and port.
        SocketAccept welcome_socket, client_socket;
        
        ! Close the welcome socket, as it is no longer needed.
        SocketClose welcome_socket;
        
    ENDPROC
    
    ! Close the connection to the client.
    PROC CloseConnection()
        SocketClose client_socket;
    ENDPROC
    
    
    ! Set ConRun to 0 if ConStat is set to 0
    PROC checkConStat()
    
        IF DI10_1 = 0 THEN
            SetDO DO10_3, 0;
            TPWrite " ConRun is set to 0 ";
        ENDIF
        
    ENDPROC
    
    ! check pose
    FUNC string readPose()
  
        VAR string currentPoseStrX;
        VAR string currentPoseStrY;
        VAR string currentPoseStrZ;
        VAR string currentPoseStr;
        VAR pos currentPose; 
        
        currentPose := CPos(\Tool:= tSCup);
        !\WObj:= wTable
        currentPoseStrX := NumToStr (currentPose.x + 5000,0);
        currentPoseStrY := NumToStr (currentPose.y + 5000,0);
        currentPoseStrZ := NumToStr (currentPose.z + 5000,0);
        
        ! make it up to the four bits format
        currentPoseStr := "0000000000000000" + currentPoseStrZ + currentPoseStrY + currentPoseStrX;
        
        RETURN currentPoseStr;
        
    ENDFUNC

    ! for returning 4 bits string ( read pose )
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
    
    ! check last 2 bits(command bits)
    FUNC num checkStatus(string value)
        VAR num status;
        VAR bool ok;
        
        value := strPart(value,29,2);
        ok := StrToVal(value, status);
        TPWrite value;
        
        RETURN status;
        
    ENDFUNC
      
    
ENDMODULE