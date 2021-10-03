clear;
%Myapp();
%function Myapp()
%fig=handle(uifigure('position',[0 0 1000 1000]));
fig=figure;
%{
ALTSLD=uislider(fig,'position',[50 990 200 3],'limits',[1000 40000],'ValueChangedFc',@(hObject,eventdata)updateALTRef(hObject,eventdata));
ALTLBL=uilabel('parent',fig,'Text','DESIRED ALTITUDE','background','g','position',[50 945 200 15]);
ALTGain=uislider(fig,'position',[50 935 200 3],'limits',[0 3],'ValueChangedFc',@(hObject,eventdata)updateALTGain(hObject,eventdata));
ALTGainLBL=uilabel('parent',fig,'Text','ALTITUDE GAIN','background','g','position',[50 890 200 15]);
VzSLD=uislider(fig,'position',[350 990 200 3],'limits',[0 300],'ValueChangedFc',@(hObject,eventdata)updateVzRef(hObject,eventdata));
VzLBL=uilabel('parent',fig,'Text','DESIRED VERTICAL SPEED','background','g','position',[350 945 200 15]);
VzP=uislider(fig,'position',[350 935 200 3],'limits',[0 1],'ValueChangedFc',@(hObject,eventdata)updateVzP(hObject,eventdata));
VzPLBL=uilabel('parent',fig,'Text','PROPORTIONAL','background','g','position',[350 890 200 15]);
VzI=uislider(fig,'position',[350 880 200 3],'limits',[0 1],'ValueChangedFc',@(hObject,eventdata)updateVzI(hObject,eventdata));
VzILBL=uilabel('parent',fig,'Text','INTEGRAL','background','g','position',[350 835 200 15]);
VzD=uislider(fig,'position',[350 825 200 3],'limits',[0 1],'ValueChangedFc',@(hObject,eventdata)updateVzD(hObject,eventdata));
VzDLBL=uilabel('parent',fig,'Text','DERIVATIVE','background','g','position',[350 780 200 15]);
%}
u2 = udp('127.0.0.1','LocalPort',49003);
fopen(u2);
%ax=uiaxes('parent',fig,'position',[0 0 1000 300]);
ax=subplot(2,2,1);
n=5;
N=100;
M=20;
hLine =handle( plot(ax,1:N,zeros(1,N)));
ylim(ax,[0 220]);
%ax.YTick=10; 
%ax2 = copyobj(ax, fig); 
%ax2=uiaxes('parent',fig,'position',[0 350 1000 300]);
ax2=subplot(2,2,2);
hLine2 = handle(plot(ax2,1:N,zeros(1,N)));
ylim(ax2,[0 1500]);
No_command=single(-999);
u = udp('127.0.0.1',49000);
fopen(u);
header=uint8([68 65 84 65 0]);
gear_brake_index=uint8([14 0 0 0]);
gear_brake_cmd=[No_command 0 No_command No_command 0 No_command No_command No_command];
throttle_cmd_index=uint8([25 0 0 0]);
throttle=0;
%throttle_cmd=[throttle throttle No_command No_command No_command No_command No_command No_command];
primary_ctr_index=uint8([11 0 0 0]);
sec_ctr_index=uint8([13 0 0 0]);
datagram=serialize(header,gear_brake_index,gear_brake_cmd);
fwrite(u,datagram);
K=[0.5 0 1.5];
altRef=2000;
IASRef=200;
Kpitch=2.5;
Kpitchrate=0.3;
Kh=1;
it=3;
pitch=[0 0];
ias=[0 0];
Integral=0;
OldstatPitch=[0 0];
while(ishandle(hLine))
    tic
   l=0;
   while(l~=5+n*36)
 A=fread(u2,5+n*9*4);
   [l dummy]=size(A);
   end
Data=zeros(n,8);
for i=1:n
    for j=2:9  
Data(i,j-1)=typecast(uint8(A(6+(i-1)*36+(j-1)*4:(i-1)*36+6+(j-1)*4+3)),'single');
    end
end
ias(it)=Data(1,1);
alt(it)=Data(3,6);%-Data(3,1);
pitch(it)=Data(2,1);
if(it>=N && mod(it,M)==0)
    %set(hLine,'ydata',ias(it-N+1:it));
    hLine.YData=ias(it-N+1:it);
   drawnow  %updates the display
elseif(it>=N && mod(it,M)==10)
    %set(hLine2,'ydata',pitch(it-N+1:it));
    hLine2.YData=alt(it-N+1:it);
   drawnow  %updates the display
end
Dpitch=[pitch(it) OldstatPitch]*[3 -4 1]';
OldstatPitch(2)=OldstatPitch(1);
OldstatPitch(1)=pitch(it);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
pitchRef=sat(Kh*(altRef-alt(it)),5,-5);

elev = sat(Kpitchrate*(Kpitch*(pitchRef - pitch(it)) - Dpitch),1,-1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Err=IASRef-ias(it);
D(it)=ias(it-2:it)*[3 -4 1]';
Integral=Integral+Err;
throttle=throttle+K*[Err;Integral;-D(it)];
if throttle>1
    throttle=1;
end
if throttle<0
    throttle=0;
end
throttle_cmd=[throttle throttle No_command No_command No_command No_command No_command No_command];
primary_ctr_cmd=[elev No_command No_command No_command No_command No_command No_command No_command];
sec_ctr_cmd=[0 No_command No_command No_command No_command No_command No_command No_command];
datagram=serialize(header,throttle_cmd_index,throttle_cmd,primary_ctr_index,primary_ctr_cmd,sec_ctr_index,sec_ctr_cmd);
fwrite(u,datagram);

    it=it+1;
    
end
fclose(u);
fclose(u2);
delete(u);
delete(u2);
function output=serialize(varargin)
[dummy n]=size(varargin);
id=1;
for i=1:n
w=typecast(varargin{i},'uint8');
[x y]=size(w);
output(id:id+y-1)=w;
id=id+y;
end
end
%{
function updateVzRef(~,~)
        IASRef=VzSLD.Value;
    end
function updateALTRef(~,~)
altRef=ALTSLD.Value;
end
function updateALTGain(~,~)
  Kh=ALTGain.Value;  
end

    function updateVzP(~,~)
        Kairspeed(1)=VzP.Value
    end
    function updateVzI(~,~)
        Kairspeed(2)=VzI.Value
    end
    function updateVzD(~,~)
        Kairspeed(3)=VzD.Value
    end
%end
%}