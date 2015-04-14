% Script to communicate with IRB120 robot system
% Mark Whitty
% 140324
%function MTRN4230_Client_Sample(str)

% The robot's IP address.
% robot_IP_address = '192.168.2.1';
robot_IP_address = '127.0.0.1';

% The port that the robot will be listening on. This must be the same as in
% your RAPID program.
robot_port = 7027;

% Open a TCP connection to the robot.

i = 1;
    socket = tcpip(robot_IP_address, robot_port);
    set(socket, 'ReadAsyncMode', 'continuous'); 

fopen(socket); 


while (i == 1)

    
    % Check if the connection is valid.
    if(~isequal(get(socket, 'Status'), 'open'))
        warning(['Could not open TCP connection to ', robot_IP_address, ' on port ', robot_port]);
    else

    
        % Read a line from the socket. Note the line feed appended to the message in the RADID sample code.
        data = fgetl(socket);

        dataNum = str2num(data);
        lastBits = rem(dataNum,100);    

        if lastBits > 90     

            dataNum = (dataNum - lastBits) / 100;
            Xpos = rem(dataNum, 10000)-5000;

            dataNum = (dataNum - Xpos)/10000;
            Ypos = rem(dataNum, 10000)-5000;

            dataNum = (dataNum - Ypos)/10000;
            Zpos = rem(dataNum, 10000)-5000;

            % Send feedback to the server on the robot.
            command = '';
            fwrite(socket, command);  
                
    
            % Print the data that we got.
            fprintf(char(data));
            fprintf('\n');

        else
             fwrite(socket, data);
        end
        pause(0.1);
    end
    
                
            % Close the socket.
   
    

    

    
end 
fclose(socket);
    
    