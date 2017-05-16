function out = NiceColours(num)

%returns a nice colourmap where each colour is dissilmiar to its predecessor

out(1,:)=[1,0,0];%red
out(2,:)=[0,1,0];%green
out(3,:)=[0,0,1];%blue
out(4,:)=[0,1,1];%cyan
out(5,:)=[1,0,1];%magenta
out(6,:)=[1,1,0];%yellow
out(7,:)=1.6*[0.35,0.35,0.35];%grey
out(8,:)=0.8*[153,51,204]/255;%purple
out(9,:)=[255,204,204]/255;%pink
out(10,:)=[255,153,51]/255;%orange
out(11,:)=[0.5,0,0];%dark red
out(12,:)=[0.75,1,0.75];%light green
out(13,:)=[248,248,255]/255;%ghost white
out(14,:)=[255,215,0]/255;%gold
out(15,:)=[25,25,112]/255;%midnight blue
out(16,:)=[195,123,43]/255;% brown
out(17,:)=[0.65,0.65,0.65];%grey
out(18,:)=[176,224,230]/255;% powder blue
out(19,:)=[221,160,221]/255;% plum
out(20,:)=[102,205,170]/255; %medium aquamarine


if exist('num')
    if length(out)>=num
        out=out(1:num,:);    
    else
        disp('Out of palette colours: generating random colours...')
%        save
        theRest=jet(num-length(out));

        theRest=theRest(randperm(size(theRest,1)),:);

        theRest=rand((num-length(out)),3);
                
        out=[out;theRest];
    end
end


