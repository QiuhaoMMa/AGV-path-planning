clc; clear; close all

maps = {
    [0 0; 0 10; 10 10; 5 10; 11 9; 4 -5];              
    [0 0; 5 0; 5 5; 10 5; 10 10];                        
    [0 0; 3 6; 6 3; 9 9; 12 6]                            
};

for i = 1:length(maps)
    disp(['Simulating map ', num2str(i)])
    simulatePathFollowing(maps{i});
end
