const
NumWheels = 4;
//ROTATE = 1;

// Global Variables
var irobot, isensor: integer;
    t, w, ki,b, Xrob, Yrob,theta: double;
    lines: TStrings;
    UDP: TStream;
    ControlMode: string;
    encoder1,encoder2,stat: integer;
    target_x, target_y, target_t, v , w1, YS, ANG2F, ERRO_theta, erro_dist:double;





procedure KeyControl(v: double);
var v1, v2: double;
begin

  //v := 10;

  //v1 := 0;
  //v2 := 0;
  if keyPressed(VK_RIGHT) then begin
    v1 := +1;
    v2 := -1;
  end;

  if keyPressed(VK_LEFT) then begin
    v1 := v1 - 1;
    v2 := v2 + 1;
  end;

  if keyPressed(VK_UP) then begin
    v1 := v1 + 1;
    v2 := v2 + 1;
  end;

  if keyPressed(VK_DOWN) then begin
    v1 := v1 - 1;
    v2 := v2 - 1;
  end;

  v1 := v1*v;
  v2 := v2*v;
  SetAxisSpeedRef(irobot, 0, v1);
  SetAxisSpeedRef(irobot, 1, v2);
end;

procedure TrackControl(v, k: double);
var  err, ys: double;
    P: TPoint3D;
begin
  P := GetSolidPos(irobot, isensor);
  if P.y > 0 then begin
    err := -P.x;
  end else if P.y > -0.25 then begin
    err := -P.x + 0.1;
  end else begin
    err := -P.x;
  end;

  //v1 := v - k * err;
  //v2 := v + k * err;

  //v1 :=  2*v - (0.1*w+2*v)/2;
  //v2 := (0.1*w+2*v)/2;


end;


procedure Control;
var ref: double;
    s: string;
    odo1,odo2: integer;
    v1,v2,sens1,sens2:double;
    dteta,d:double;



begin

  if keyPressed(ord('R')) then begin
    SetRobotPos(irobot, 0, 0.4, 0, 0);

    encoder1 := 0;
    encoder2 := 0;
    ki:= 0.0005;
    b:=0.1;

  end;

  if keyPressed(ord('S')) then begin
    ControlMode := 'keys';
  end else if keyPressed(ord('T')) then begin
    ControlMode := 'track';
  end else begin
    //ControlMode := 'keys';
  end;

  if ControlMode = 'keys' then begin
    KeyControl(10);
  end;

  t := t + 0.04;
  if w*t >= 2*pi then begin
    t := t - 2*pi/w;
  end;

  if controlMode = 'track' then begin
    TrackControl(10, 150);
  end;

  sens1:=GetSensorVal(0,0);
  sens2:=GetSensorVal(0,1);


  //GetAxisOdo(RobotIndex, AxisIndex);
  odo1:=GetAxisOdo(0,0);
  odo2:=GetAxisOdo(0,1);

  SetRCValue(1, 1 ,format('%.3g',[sens1]));
  SetRCValue(2, 1 ,format('%.3g',[sens2]));

  SetRCValue(1,3,format('%d',[Odo1]));
  SetRCValue(2,3,format('%d',[Odo2]));


  encoder1 := encoder1+Odo1;
  encoder2 := encoder2+Odo2;

  SetRCValue(1,4,format('%d',[encoder1]));
  SetRCValue(2,4,format('%d',[encoder2]));


  dteta := (odo2*ki - odo1*ki) / b  ;

  d := (odo1*ki + odo2*ki)/2 ;




  Xrob :=  Xrob + d* cos(theta+dteta/2);
  Yrob:=  Yrob + d* sin(theta+dteta/2);


  theta:= NORMALIZEANGLE (theta+dteta) ;


  SetRCValue(1,7,format('%f',[Xrob]));
  SetRCValue(2,7,format('%f',[Yrob]));
  SetRCValue(3,7,format('%f',[theta*180/pi]));


end;

 Procedure  goTOxyg;
 
var v1,v2,sens1,sens2:double;
    dteta,d:double;
    
 begin
 
    if keyPressed(ord('G')) then begin
    SetRobotPos(irobot, 0, 0.4, 0, 0);

    encoder1 := 0;
    encoder2 := 0;
    ki:= 0.0005;
    b:=0.1;

  end;
  
  
    target_x:= GetRCValue(6, 8);
    target_y:= GetRCValue(7, 8);
    target_t:= atan2(target_y,target_x);
   // target_t:=(target_t*pi )/ 180;

    SetRCValue(8,8,format('%f',[target_t]));

  //  v:=  GetRCValue(10, 8);
  // w1:=  GetRCValue(11,8);

    ANG2F:= atan2(target_y-yrob,target_x-xrob);
    //ANG2F:= atan2(target_y-0.4,target_x-0);
    Erro_theta := -normalizeangle (ang2f - theta);
    Erro_dist:= SQRT( (target_x-xrob)*(target_x-xrob) + (target_y-yrob)*(target_y-yrob));





     case stat of

  1: begin   //rotate
      if (abs(erro_theta) < 0.03) then begin
      stat:=2;
      end;

  end;

  2: begin        //go forward
  if (abs(erro_dist) < 0.05) then begin
      stat:=3;
      end;

  if (abs(erro_theta) >0.5) then begin
      stat:=1;
      end;
  end;

  3: begin      //de_accel

  if (erro_dist < 0.02) then begin
      stat:=4;
      end;

  end;

  4: begin      //Final_ROT

  //if (abs(erro_theta) < 0.5) then begin
      stat:=5;
    //  end;

  end;

  5: begin  //de_acc_final_rot

 //if (abs(erro_theta) < 0.2) then begin
      stat:=6;
   //   end;


  end;

  6: begin     //stop
   //   stat:=1;
  end;

end;  //end case

////////////////////////////////////////////////////////////////


case stat of

1:  begin
      v:=0;
      w:= 25;
    end;
2: begin
      v:=5;
      w:= 5*erro_theta;
  end;
3: begin
      v:=0.5;
      w:=5*erro_theta;
    end;
4: begin
      v:=0;
      w:=2;
    end;

5: begin
      v:=0;
      w:= 0.5;
    end;

6:begin
      v:=0;
      w:=0;
  end;

end;


//v1 :=  2*v - (0.1*w+2*v)/2;
//v2 := (0.1*w+2*v)/2;

 v1 := v-w*b/2;
 v2 := v+w*b/2;


SetAxisSpeedRef(irobot, 0, v1);
SetAxisSpeedRef(irobot, 1, v2);

SetRCValue(7,2,format('%d',[stat]));
SetRCValue(8,2,format('%g',[abs(erro_theta)*180/pi]));
SetRCValue(11,2,format('%g',[w]));
SetRCValue(12,2,format('%g',[ANG2F*180/pi]));
SetRCValue(14,2,format('%g',[dteta*180/pi]));
SetRCValue(15,2,format('%g',[erro_dist]));

end;

procedure Initialize;
begin
  irobot := 0;
  isensor := GetSolidIndex(irobot, 'NXTLightSensor');

  t := 0;
  w := 1;
    ki:= 0.0005;
    b:=0.1;
    theta:=-pi/2 ;
    stat:=1;

  Xrob :=  0;
  Yrob  := 0.4;


  ControlMode := 'keys';
end;
