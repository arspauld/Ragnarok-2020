
function[hm]=altitude_conversion(Psta)
format long
%halt= pressure altitude (ft)
%psta=station pressure (mb)

halt=(1-(Psta/1013.25)^0.190284)*145366.45


%hm= pressure altitude (m)
hm=0.3048*halt
