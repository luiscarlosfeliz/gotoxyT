const NumWheels = 4;

// Global Variables
var irobot, isensor: integer;
    t, w: double;
    lines: TStrings;
    UDP: TStream;
    ControlMode: string;
    encoder1, encoder2, xyz, kimp, dteta, tetaAnt, delta_d, delta_t, b, x_act, y_act, t_act, tfGT,xfGT,yfGT: double;
    estadoGT,estadoSQ, flag,flag1,flag2, estadoFL:integer;
    ang_ida, erro_ang_gt, erro_angf, x_fl_ir, y_fl_ir,distx, disty,x2fl,y2fl,x1fl,y1fl,tfl, xi, xf, yi, yf : double;
    
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

  procedure Fline(xi,yi,xf,yf,tfl: double);

var   ang_fl_ir, apx, apy, apnx, apny: double;
    nx, ny, dist, Ampx, Ampy, prod_int, raiz, x_reta, y_reta,  erroFLang,kfl1, kfl2: double;

begin

SetRCValue(19, 1 ,format('%s',['estadoFL=']));  SetRCValue(19, 2 ,format('%d',[estadoFL]));

  SetRCValue(10, 6 ,format('%s',['Xi= ',xi]));
  SetRCValue(11, 6 ,format('%s',['Yi= ',yi]));
  SetRCValue(12, 6 ,format('%s',['xF= ',xf]));
  SetRCValue(13, 6 ,format('%s',['yF= ',yf]));
  SetRCValue(14, 6 ,format('%s',['tfl= ', tfl]));

  (*xi := GetRCValue(10, 7 );
  yi := GetRCValue(11, 7 );
  xf := GetRCValue(12, 7 );
  yf := GetRCValue(13, 7 );
  tfl := GetRCValue(14, 7 )*pi/180;    *)


  (*nx:=xf-xi;
  ny := yf-yi;

  raiz:= SQRT( (xf-xi)*(xf-xi) + (yf-yi)*(yf-yi));


  //calculo do versor n que segue a direçao da linha
  nx:=nx/raiz;
  ny:= ny/raiz;

  //representam o vector a-p
  AmPx:= xi-x_act;
  Ampy := yi-y_act;

  distx:= AmPx;
  disty:= AmPy;

  // calculo auxiliar do produto interno do vector a-p com o versor n
  prod_int := AmPx*nx + AmPy*ny;

  //distancias em x e em y
  distX := distX + prod_int*nx;
  distY:= distY + prod_int*ny ;

   SetRCValue(17, 8 ,format('%s',['distX=']));  SetRCValue(17, 9 ,format('%g',[distX]));
        SetRCValue(18, 8 ,format('%s',['distY=']));  SetRCValue(18, 9 ,format('%g',[distY]));

  //a distancia vai ser a norma do vector anterior

  dist := SQRT(distX*distX + distY*distY);

  //este será o ponto da recta mais proximo do robot se este seguir numa direcção perpendicular
  x_reta := distX+x_act;
  y_reta := distY+y_act;

  SetRCValue(17, 10 ,format('%s',['x_reta=']));  SetRCValue(17, 11 ,format('%g',[x_reta]));
        SetRCValue(18, 10 ,format('%s',['y_reta=']));  SetRCValue(18, 11 ,format('%g',[y_reta]));
              SetRCValue(15, 6 ,format('%s',['dist=']));  SetRCValue(15, 7 ,format('%g',[dist]));
            *)




           

   //que raio é que está diferente aqui????
  nx := xf-xi;
  ny := yf-yi;
  raiz :=  sqrt(nx*nx+ny*ny);
  nx := nx / raiz;
  ny := ny / raiz;
  apx := xi-x_act;
  apy := yi-y_act;
  prod_int := apx*nx+apy*ny;
  apnx := prod_int*nx;
  apny := prod_int*ny;
  dist := sqrt( ((apx-apnx)*(apx-apnx)) + ((apy-apny)*(apy-apny) ));
  x_reta := (apx - apnx)+x_act;
  y_reta := (apy - apny)+y_act;
  
    SetRCValue(15, 6 ,format('%s',['dist=']));  SetRCValue(15, 7 ,format('%g',[dist]));





  //////////////////////////////////////////////////////////////////////***********************************************|\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

 case estadoFL OF
    0:begin
      if((abs(dist))>0.05)then begin
        x_fl_ir:=x_reta;
        y_fl_ir:=y_reta;
        estadoFL:=1;
        estadoGT:=0;
      end
      else begin
        x_fl_ir:=xf;
        y_fl_ir:=yf;
        estadoFL:=2;
      end
	  end;
     1:begin
     (*RODA e VAI PARA A PERPENDICULAR*)


	    ang_fl_ir:=atan2(yf-y_fl_ir,xf-x_fl_ir); // angulo que tem de estar quando chegar a reta

      SetRCValue(17, 6 ,format('%s',['x_fl_ir=']));  SetRCValue(17, 7 ,format('%g',[x_fl_ir]));
        SetRCValue(18, 6 ,format('%s',['y_fl_ir=']));  SetRCValue(18, 7 ,format('%g',[y_fl_ir]));
          SetRCValue(19, 6 ,format('%s',['ang_fl_ir=']));  SetRCValue(19, 7 ,format('%g',[ang_fl_ir*180/pi]));

      GoTo_xyt(x_fl_ir,y_fl_ir,ang_fl_ir);
      if(dist<0.1)then
      begin
        estadoFL:=2; estadoGT:=0;
      end;
    end;
    2:begin
      erroflang:=  -normalizeangle(ang_fl_ir-t_act);
      VelocidadeGoTo(4,kfl1*dist+Kfl2*erroflang);
      if(dist<0.08)then
      begin
        estadoFL:=3; estadoGT:=0;
      end;
    end;
    3:begin
    (*SEGUE LINHA*)
     GoTo_xyt(xf,yf,tfl);
     if(abs(dist)>0.1)then begin estadoFL:=0;end;
     if((abs(xf-x_act)<0.03) and (abs(yf-y_act)<0.03)and(abs(-normalizeangle(tfl-t_act))<0.07))then
     begin estadoFL:=4;end;

    end;
    4:begin
    (*PARADO*)
	end;
  end;


       ////////////////////////////////////////////////////////////////****************************************|\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


end;





//////////////////////////QUADRADO\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
procedure fSquare;
var  gg:double;sens1,sens2:double;
    parede0x,parede1y,parede2x,parede3y, ang_new:double;
    y_act_new:double;
begin

  sens1:=GetSensorVal(0,0);
  sens2:=GetSensorVal(0,1);
  case estadoSQ OF
    0:begin
       if (( sens1 > -1 ) and (sens2 > -1)) then begin
          ang_new:= -normalizeangle(atan2(sens1-sens2,0.1));
           SetRCValue(13, 1 ,format('%s',['ang_new=']));SetRCValue(13,2  ,format('%g',[ang_new*pi/180]));

          t_act := ang_new;
          
             //0.20 parece ser a distancia da parede  a y=1
           y_act:= (0.20-((sens1+sens2)/2)*cos(ang_new)) + 1;
      end;

      //ele vai do (0,0) para o (0,1) e este movimento inclui a rotação para a próxima linha
      FLine(0,0,0,1,0);
      

      if((abs(0-x_act)<0.04) and (abs(1-y_act)<0.04))then
          begin

          estadoSQ:=1;estadoFL:=0; end;
         end;
         


    1:begin

     if (( sens1 > -1 ) and (sens2 > -1)) then begin
          ang_new:= -normalizeangle(atan2(sens1-sens2,0.1));
           SetRCValue(13, 1 ,format('%s',['ang_new=']));SetRCValue(13,2  ,format('%g',[ang_new*pi/180]));

          t_act := ang_new;
            //0.20 parece ser a distancia da parede  a y=1
           y_act:= (0.20-((sens1+sens2)/2)*cos(ang_new)) + 1;
           

      end;

      FLine(0,1,1,1,-90);
      
      if((abs(1-x_act)<0.04) and (abs(1-y_act)<0.04))then
      begin estadoSQ:=2;estadoFL:=0; end;

       end;
    2:begin
             if (( sens1 > -1 ) and (sens2 > -1)) then begin
          ang_new:= -normalizeangle(atan2(sens1-sens2,0.1));
           SetRCValue(13, 1 ,format('%s',['ang_new=']));SetRCValue(13,2  ,format('%g',[ang_new*pi/180]));

        t_act := ang_new - (pi/2);
        
       x_act:= (0.20-((sens1+sens2)/2)) * cos( ang_new - (pi/2) ) + 1;
         
      end;


      FLine(1,1,1,0,-180);
      
       if((abs(1-x_act)<0.04) and (abs(0-y_act)<0.04))then
      begin estadoSQ:=3;estadoFL:=0; end;


    end;
    3:begin
            if (( sens1 > -1 ) and (sens2 > -1)) then begin
          ang_new:= -normalizeangle(atan2(sens1-sens2,0.1));
           SetRCValue(13, 1 ,format('%s',['ang_new=']));SetRCValue(13,2  ,format('%g',[ang_new*pi/180]));

          t_act := ang_new - (pi/2);
          
           x_act:= (0.20-((sens1+sens2)/2)) * cos( ang_new - (pi/2) ) + 1;
      end;

      FLine(1,0,0,0,90);
      
       if((abs(0-x_act)<0.04) and (abs(0-y_act)<0.04))then
      begin estadoSQ:=0;estadoFL:=0; end;

    end;
  end;
  (*
  SetRCValue(30, 1 ,format('%s',['estadoDDQ=']));SetRCValue(30, 2 ,format('%d',[estadoDDQ]));
  SetRCValue(31, 1 ,format('%s',['voltasDDQ=']));SetRCValue(31, 2 ,format('%d',[voltasDDQ]));
  SetRCValue(30, 3 ,format('%s',['xxx']));SetRCValue(30,4 ,format('%f',[xxx*180/pi]));      *)

  SetRCValue(5, 8 ,format('%s',['sensor1=']));SetRCValue(5,9  ,format('%g',[sens1]));
  SetRCValue(6, 8 ,format('%s',['sensor2=']));SetRCValue(6,9  ,format('%g',[sens2]));

end;


//////////////////////////////////QUADRADO\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\



procedure Control;

var ref: double;
    s: string;
    odo1,odo2: integer;
    sens1,sens2:double;
begin

  if keyPressed(ord('R')) then begin
    SetRobotPos(irobot, 0, 0, 0, 0);     //g           rrrgr
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


  SetRCValue(11, 1 ,format('%s',['y_act=']));
  SetRCValue(11, 2 ,format('%.3g',[y_act]));


  SetRCValue(12, 1 ,format('%s',['theta_act=']));
  SetRCValue(12, 2 ,format('%.3g',[t_act*180/pi]));


  //VelocidadeManual();
    SetRCValue(4,4 ,format('%s',['xF']));
    SetRCValue(5, 4 ,format('%s',['yF']));
    SetRCValue(6, 4 ,format('%s',['ThetaF']));
  //Verificar se Botao foi precionado

  if( keyPressed(ord('G')) =true)then begin

   flag:=1;
   estadoGT:=0;
    //OBTER xf e yf e tf
    


    xfGT:=GetRCValue(4,5);
    yfGT:=GetRCValue(5,5);
    tfGT:=GetRCValue(6,5)*pi/180;
    

  end;

    SetRCValue(4,6 ,format('%g',[xFGT]));
    SetRCValue(5, 6 ,format('%g',[yFGT]));
    SetRCValue(6, 6 ,format('%g',[tfGT*180/pi]));
    SetRCValue(7, 4 ,format('%s',['flag']));
    SetRCValue(7, 5 ,format('%d',[flag]));


   if(flag=1) then begin
   GoTo_xyt(xfGT,yfGT,tfGT);
   end;

  if( keyPressed(ord('F')) =true)then begin
   flag1:=1;
  xi := GetRCValue(10, 7 );
  yi := GetRCValue(11, 7 );
  xf := GetRCValue(12, 7 );
  yf := GetRCValue(13, 7 );
  tfl := GetRCValue(14, 7 )*pi/180;
   
   end;
   
    if(flag1=1) then begin
    //executar os followline pela seguinte ordem para fazer um quadrado
    fline(xi,yi,xf,yf,tfl);

    end;
    

      if( keyPressed(ord('Q')) =true)then begin
   flag2:=1; end;
   
   if(flag2=1) then begin

    fSquare;

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
  Y_act  := 0;
  
  ControlMode := 'keys';
end;
