yolo = [1 2 0 1 0 1 2 0 1 0 1 2 0 1 0 1 2 0 1 0 1 2 0 1 0 1 2 0 1 0];
mini = 3;
maxi = 7;

counter = 0;

for n = maxi : -1 : mini
 
  for i = length(yolo)-n : -1 : n
    
    for j = 1 : n
      if yolo(j) == yolo(j+i)
        counter = counter + 1;
      end
    end
    
    if counter == n && i == n
      display('Period has been found. Period length: ');
      display(n);
    else
      counter = 0;
    end
    
  end
  
end


