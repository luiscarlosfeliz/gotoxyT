const NumWheels = 4;

// Global Variables
var irobot, isensor: integer;
    t, w: double;
    lines: TStrings;
    UDP: TStream;
    ControlMode: string;
    encoder1, encoder2, xyz, kimp, dteta, tetaAnt, delta_d, delta_t, b, x_act, y_act, t_act, xf, yf, tf: double;
    estadoGT, flag:integer;
    ang_ida, erro_ang_gt, erro_angf: double;
procedure KeyControl(v: double);
var v1, v2: double;
begin

  v := 10;

  v1 := 0;
  v2 := 0;
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
var v1, v2, err, ys: double;
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
  
  v1 := v - k * err;
  v2 := v + k * err;

  SetAxisSpeedRef(irobot, 0, v1);
  SetAxisSpeedRef(irobot, 1, v2);
end;


procedure VelocidadeManual;
var  v1,v2,vl,vw: double;
 begin
  SetRCValue(4, 1 ,format('%s',['v=']));
  vl:=GetRCValue(4,2);
  SetRCValue(5, 1 ,format('%s',['w(graus/s)=']));
  vw:=GetRCValue(5,2)*pi/180;

  v1 := vl+vw*b/2;
  v2 := vl-vw*b/2;

  SetAxisSpeedRef(irobot, 0, v1);
  SetAxisSpeedRef(irobot, 1, v2);

end;

procedure VelocidadeGoTo(vgt,wgt:double);
var  v1,v2: double;
 begin

  v1 := vgt+wgt*b/2;
  v2 := vgt-wgt*b/2;

  SetAxisSpeedRef(irobot, 0, v1);
  SetAxisSpeedRef(irobot, 1, v2);


end;

procedure GoTo_xyt(xff,yff,tff:double);
(*funcao que vai para xff yff e teta_ff*)
var
     k1,k2,k3:double;
begin
  k1:=40; k2:=40;k3:=50;
  ang_ida:= atan2(yff-y_act,xff-x_act);
  erro_ang_gt:=-normalizeangle(ang_ida-t_act);
  erro_angf:=-normalizeangle(tff-t_act);

  case estadoGT Of
    0:begin
    (*RODA PARA IR*)
      //if(abs(erro_ang_gt)>0.05)then begin k1:=50;end;
      VelocidadeGoTo(0,k1*erro_ang_gt);
      if(abs(erro_ang_gt)<0.03)then begin estadoGT:=1;end;
    end;
     1:begin
     (*VAI PARA o PONTO*)
      if(abs(erro_ang_gt)>0.05)then begin estadoGT:=0;end;
      VelocidadeGoTo(10,k2*erro_ang_gt);
      if((abs(xff-x_act)<0.02) and (abs(yff-y_act)<0.02))then begin estadoGT:=2;end;
    end;
    2:begin
    (*RODA PARA O ANG FINAL*)
        VelocidadeGoTo(0,k3*erro_angf);
        if( abs(erro_angf)<0.05 )then
        begin estadoGT:=3; flag:=0; end;
    end;
    3:begin
    (*PARADO*)
    end;
  end;

  SetRCValue(18, 1 ,format('%s',['estadoGT=']));  SetRCValue(18, 2 ,format('%d',[estadoGT]));
  SetRCValue(15, 3 ,format('%s',['ang_ida=']));  SetRCValue(15, 4 ,format('%f',[ang_ida*180/pi]));
  SetRCValue(16, 3 ,format('%s',['t_Act=']));  SetRCValue(16, 4 ,format('%f',[t_act*180/pi]));
  SetRCValue(17, 3 ,format('%s',['erro_ang=']));  SetRCValue(17, 4 ,format('%f',[erro_ang_gt*180/pi]));
  SetRCValue(18, 3 ,format('%s',['erro_angf=']));  SetRCValue(18, 4 ,format('%f',[erro_angf]));
end;

Procedure Fline;

Var



begin
 SetRCValue(4, 7 ,format('%s',['x1F=']));  SetRCValue(4, 8 ,format('%d',[x1f]));
 SetRCValue(5, 7 ,format('%s',['y1F=']));  SetRCValue(5, 8 ,format('%d',[y1f]));
 SetRCValue(6, 7 ,format('%s',['x2F=']));  SetRCValue(6, 8 ,format('%d',[x2f]));
 SetRCValue(7, 7 ,format('%s',['y2F=']));  SetRCValue(8, 8 ,format('%d',[y2f]));


 n1 := (x2f-x1f)/(sqrt(((x2f-x1f)*(x2f-x1f))+((y2f-y1f)*(y2f-y1f))));
 n2 := (y2f-y1f)/(sqrt(((x2f-x1f)*(x2f-x1f))+((y2f-y1f)*(y2f-y1f))));

 A:= (x1f-x_act)*n1 + (y1f-y_act)*n2;

 xp := x_act + (x1f-x_act) - (A*n1);
 yp := y_act + (y1f-y_act) - (A*n2);

 edlinha := sqrt(Norm2(xp-x_act,yp-y_Act));
 edpfinal := sqrt(Norm2(x2-x_act,y2-y_act));

 alinha1 := atan2(yp-y_act, xp-x_act);
 ealinha1 := normalizeangle(t_act-alinha1);

 alinha2 := atan2(y2f-y_act, x2f-x_Act);
 ealinha2 := normalizeangle(t_Act-alinha2);
 



end;



procedure Control;

var ref: double;
    s: string;
    odo1,odo2: integer;
    sens1,sens2:double;
begin

  if keyPressed(ord('R')) then begin
    SetRobotPos(irobot, 0, 0.4, 0, 0);
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

  //Fazemos o robo andar sempre em frente: v=20 e w=0 e chegamos ao valor de kimp_est~=0.0005
  odo1:=GetAxisOdo(0,0);
  odo2:=GetAxisOdo(0,1);

  //SetRCValue(4, 1 ,format('%s',['odo1=']));
  //SetRCValue(4, 2 ,format('%d',[odo1]));
  //SetRCValue(5, 1 ,format('%s',['odo2=']));
  //SetRCValue(5, 2 ,format('%d',[odo2]));

  encoder1 := encoder1+Odo1;
  encoder2 := encoder2+Odo2;

  //SetRCValue(7, 1 ,format('%s',['encoder1=']));
  //SetRCValue(7, 2 ,format('%.3g',[encoder1]));
  //SetRCValue(8, 1 ,format('%s',['encoder2=']));
  //SetRCValue(8, 2 ,format('%.3g',[encoder2]));

  //Estimar kimp indo sempre em frente
  xyz:=encoder1+encoder2;
  if ( xyz=0) then begin xyz:=0.000001  end;
  //SetRCValue(7, 3 ,format('%s',['kimp_est=']));
  //SetRCValue(7, 4 ,format('%.3g',[2* GetRobotY(irobot)/(xyz)]));


//bbbbbbbbbbbbbbbbbbbb)
  //Basta no sitio em que estamos rodar um pouco e chegamos ao valor de b_est~=0.1
  //Estimar b rodando
  xyz:= kimp*(encoder2-encoder1);
  dteta:=GetRobotTheta(irobot)-tetaant;
  if ( xyz=0) then begin xyz:=0.000001  end;
  //SetRCValue(8, 3 ,format('%s',['b_est=']));
  //SetRCValue(8, 4 ,format('%.3g',[xyz/(dteta)]));

//ccccccccccccccccccccccc
//usar o kimp dado e b dado podemos agora axar o delta_d e o delta_t
  delta_d:=( kimp*odo1+ kimp*odo2)/2;
  delta_t:=(kimp*odo2-kimp*odo1)/b;

  //Calculo proximo
   x_act:=x_act+(delta_d)*cos(t_act+(delta_t)/2);
   y_act:=y_act+(delta_d)*sin(t_act+(delta_t)/2);
   t_act:=NORMALIZEANGLE(t_act+(delta_t));

  SetRCValue(10, 1 ,format('%s',['x_act=']));
  SetRCValue(10, 2 ,format('%.3g',[x_act]));
  SetRCValue(10, 3 ,format('%s',['irobot_x']));
  SetRCValue(10, 4 ,format('%.3g',[ GetRobotX(irobot)]));

  SetRCValue(11, 1 ,format('%s',['y_act=']));
  SetRCValue(11, 2 ,format('%.3g',[y_act]));
  SetRCValue(11, 3 ,format('%s',['irobot_y']));
  SetRCValue(11, 4 ,format('%.3g',[  GetRobotY(irobot)]));

  SetRCValue(12, 1 ,format('%s',['theta_act=']));
  SetRCValue(12, 2 ,format('%.3g',[t_act*180/pi]));
  SetRCValue(12, 3 ,format('%s',['irobot_theta']));
  SetRCValue(12, 4 ,format('%.3g',[ GetRobotTheta(irobot)*180/pi]));

  //VelocidadeManual();
    SetRCValue(4,4 ,format('%s',['xF']));
    SetRCValue(5, 4 ,format('%s',['yF']));
    SetRCValue(6, 4 ,format('%s',['ThetaF']));
  //Verificar se Botao foi precionado

  if( keyPressed(ord('G')) =true)then begin

   flag:=1;
   estadoGT:=0;
    //OBTER xf e yf e tf
    


    xf:=GetRCValue(4,5);
    yf:=GetRCValue(5,5);
    tf:=GetRCValue(6,5)*pi/180;
    

  end;

    SetRCValue(4,6 ,format('%g',[xF]));
    SetRCValue(5, 6 ,format('%g',[yF]));
    SetRCValue(6, 6 ,format('%g',[tf*180/pi]));
    SetRCValue(7, 4 ,format('%s',['flag']));
    SetRCValue(7, 5 ,format('%d',[flag]));


   if(flag=1) then begin
   GoTo_xyt(xf,yf,tf);
   end;

end;



procedure Initialize;
begin
  irobot := 0;
  isensor := GetSolidIndex(irobot, 'NXTLightSensor');
  
  t := 0;
  w := 1;
  
  kimp:= 0.0005;
  b:=0.1;
  t_act:=-pi/2 ;

  estadoGT :=0;
  X_act :=  0;
  Y_act  := 0.4;
  
  ControlMode := 'keys';
end;
