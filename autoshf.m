clear all;

for n = 0 : 7;

  fid1 = fopen(['0000000' num2str(n) '.CSV']);
  asd = textscan(fid1, '%s', 'Delimiter', ';');
  fclose(fid1);
  fasd = [asd{:}];
  data=cellfun(@str2num,fasd(7:end)); 

  time = data(1 : 6 : end);
  accX = data(2 : 6 : end);

  fid2 = fopen(['slog' num2str(n) '.txt']);
  asd2 = textscan(fid2, '%s', 'Delimiter', ' ');
  fclose(fid2);
  fasd2 = [asd2{:}];
  dstate = fasd2(5 : 7 : end);
  dtime = fasd2(7:7:end);
  VarName7=cellfun(@str2num,dtime);
  VarName5=cellfun(@str2num,dstate);

  valueS = zeros(length(time),1);
  for i = 1 : length(time)
    for j = 1 : (length(VarName7)-2)
      if time(i) >= VarName7(j) && time(i) <= VarName7(j+1)
        valueS(i) = VarName5(j);
      end
      if time(i) > VarName7(length(VarName7)-1)
        valueS(i) = VarName5(length(VarName7)-1);
      end
    end
  end

  figure(n+1);
  subplot(2, 1, 1);
  plot(time, accX);
  title('Acceleration X');
  subplot(2,1,2);
  plot(time, valueS);
  set(gca, 'YTick',0:2, 'YTickLabel',{'Left' 'Right' 'Straight'});
  set(gca, 'ylim', [-0.3 2.3]);
  title('States');

end