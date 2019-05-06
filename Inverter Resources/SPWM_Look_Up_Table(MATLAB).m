%% Find SPWM lookup table

time = 0;
Array = 1;
% SPWM = transpose(SPWM);
SPWM = SPWM;
[s ~] = size(SPWM);

for n = 2:1:s
    
if (SPWM(n-1) ~= SPWM(n))
    
        Array = Array + 1;
        time(Array) = 0;
        
end
        
if (SPWM(n) == 0)
    
    time(Array) = time(Array) + 1;
    
end
    
if (SPWM(n) == 1)
    
    time(Array) = time(Array) + 1;
    
end

end

% % Save lookup table on Windows
%fileID = fopen('C:\Users\Olive\OneDrive\Desktop\lookup_10uS.txt','w');

 % Save lookup table on Machintosh
fileID = fopen('/Users/oliverrodriguez/Desktop/lookup_100uS.txt','w');
fprintf(fileID,'%d,\n', time);        
