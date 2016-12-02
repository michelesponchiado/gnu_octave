function [xyzace_post_rtcp] = rtcp_apply(xyzace_pre_rtcp)
  xyzace_post_rtcp = xyzace_pre_rtcp;
  % Macro che permette di passare da gradi a radianti
  defDeg2Rad = pi / 180.0;
  % Macro che permette di passare da radianti a gradi
  defRad2Deg = 1.0 / defDeg2Rad;
  
  % current tool floor quote
  fQuotaPiano_mm = 468.967438;
  % pidpar.ini::QPTF2mm flo mil 179.979996
  QPTF2mm = 179.979996;
  % pidpar.ini::AsseXmm
  fPosizioneCentroRotazioneX_mm = 310.933990;
  % pidpar.ini::AsseYmm
  fPosizioneCentroRotazioneY_mm = 257.444000;
  % pidpar.ini::AsseQPmm flo mil 125.000000
  fDistanzaAsseDivisorePianoMm = 125.0;
  % as in customer's configuration
  ui_cfg_45_asse = 3;
  sign_A = 1;
  if (ui_cfg_45_asse == 3)
    sign_A = -1;
  endif
  fPosizioneCentroRotazioneY_sign_OK_mm = -1 * fPosizioneCentroRotazioneY_mm;
  
  defIndexAsseX = 1;
  defIndexAsseY = 2;
  defIndexAsseZ = 3;
  defIndexAsseA = 4;
  defIndexAsseC = 5;
  defIndexAsseTg = 7;
  l_pos_before_tcp_imp = zeros (8,1);
  l_pos_before_tcp_imp(defIndexAsseX) = xyzace_pre_rtcp(1);
  l_pos_before_tcp_imp(defIndexAsseY) = xyzace_pre_rtcp(2);
  l_pos_before_tcp_imp(defIndexAsseZ) = xyzace_pre_rtcp(3);
  l_pos_before_tcp_imp(defIndexAsseA) = xyzace_pre_rtcp(4);
  l_pos_before_tcp_imp(defIndexAsseC) = xyzace_pre_rtcp(5);
  dblMmImp = ones (8,1);
  dblMmImp(defIndexAsseX) = 499.975006;
  dblMmImp(defIndexAsseY) = 499.987488;
  dblMmImp(defIndexAsseZ) = 499.975006;
  dblImpMm = ones (8,1);
  dblImpMm(defIndexAsseX) = 1.0 / dblMmImp(defIndexAsseX);
  dblImpMm(defIndexAsseY) = 1.0 / dblMmImp(defIndexAsseY);
  dblImpMm(defIndexAsseZ) = 1.0 / dblMmImp(defIndexAsseZ);
  
  dblDeg2Imp = ones (8,1);
  dblDeg2Imp(defIndexAsseA) = 3000;
  dblDeg2Imp(defIndexAsseC) = 3000;
  dblDeg2Imp(defIndexAsseTg) = 1.422222;

  dblImp2Deg = ones (8,1);
  dblImp2Deg(defIndexAsseA) = 1.0 / dblDeg2Imp(defIndexAsseA);
  dblImp2Deg(defIndexAsseC) = 1.0 / dblDeg2Imp(defIndexAsseC);
  dblImp2Deg(defIndexAsseTg) = 1.0 / dblDeg2Imp(defIndexAsseTg);

  dblOffsetImpulsi = zeros (8,1);
  dblOffsetImpulsi(defIndexAsseA) = 144731.000000;
  dblOffsetImpulsi(defIndexAsseC) = 16737.000000;

  
  
  % calcolo le coordinate di A e C in radianti
  % posizione asse A in radianti
  fPosRad_0=(l_pos_before_tcp_imp(defIndexAsseA) - dblOffsetImpulsi(defIndexAsseA)) * dblImp2Deg(defIndexAsseA) * defDeg2Rad;
  % posizione asse C in radianti
  fPosRad_1=(l_pos_before_tcp_imp(defIndexAsseC) - dblOffsetImpulsi(defIndexAsseC)) * dblImp2Deg(defIndexAsseC) * defDeg2Rad;

%
%
% BEGIN TANGENTIAL AXIS HERE    
%
%
  %// get the current tangential axis position [pulses]
  rtcp_tangential_axis.f_input_alfa_asse_tg_pulses = l_pos_before_tcp_imp(defIndexAsseTg);
  %// now we get the anticlockwise positive position in radiants, without the position offset
  %// invert the sign because the tangential axis move positive clockwise, convert from pulses to degrees, remove the axis offset, convert from degrees to radiants
  rtcp_tangential_axis.f_alfa_asse_tg_no_offset_rad = -1  * (rtcp_tangential_axis.f_input_alfa_asse_tg_pulses * dblImp2Deg(defIndexAsseTg) - dblOffsetImpulsi(defIndexAsseTg))* (defDeg2Rad);
  %// the RTCP calculation needs to be done
  rtcp_tangential_axis.do_calc = 1;
%
%
% END TANGENTIAL AXIS HERE    
%
%
  
  
 
  % calcolo la distanza attuale fra punta dell'utensile ed asse del divisore
  % quota piano utensile-quota z attuale-->distanza utensile dal piano
  % distanza utensile dal piano-distanza asse divisore dal piano-->distanza punta dell'utensile dall'asse del divisore
  f_base_distanza_tool_asse_divisore_mm_z = fQuotaPiano_mm - l_pos_before_tcp_imp(defIndexAsseZ) * dblImpMm(defIndexAsseZ) - fDistanzaAsseDivisorePianoMm;
  % coordinata attuale x
  f_act_x_coordinate_mm = l_pos_before_tcp_imp(defIndexAsseX) * dblImpMm(defIndexAsseX);
  % valore assoluto della coordinata attuale y
  f_act_y_coordinate_mm = l_pos_before_tcp_imp(defIndexAsseY) * dblImpMm(defIndexAsseY);
  if (f_act_y_coordinate_mm > 0)
    error("y >0 !");
  endif;
  % applico la rotazione C immaginando che asse A sia verticale;  dopo questa correzione vado ad applicare
  % il brandeggio A
  f_C_rot_angle = fPosRad_1;
  
  
%
%
% BEGIN TANGENTIAL AXIS HERE    
%
%
	if (rtcp_tangential_axis.do_calc)
		rtcp_tangential_axis.f_alfa_asse_tg_no_offset_C_rot_rad = rtcp_tangential_axis.f_alfa_asse_tg_no_offset_rad + f_C_rot_angle;
	endif
%
%
% END TANGENTIAL AXIS HERE    
%
%
  

  %#ifdef def_rtcp_with_tangential_axis
  %add_calc_rtcp_tangential_axis_C_rotation(f_C_rot_angle);
  %#endif			

  cos_C = cos(f_C_rot_angle);
  sin_C = sin(f_C_rot_angle);
  rx = (f_act_x_coordinate_mm - fPosizioneCentroRotazioneX_mm);
  ry = (f_act_y_coordinate_mm - fPosizioneCentroRotazioneY_sign_OK_mm);
  f_new_x_coordinate_mm = rx * cos_C - ry * sin_C + fPosizioneCentroRotazioneX_mm;
  f_new_y_coordinate_mm = rx * sin_C + ry * cos_C + fPosizioneCentroRotazioneY_sign_OK_mm;
  
  
  % distanza Y fra la posizione Y corrente e l'asse del divisore
  % imposto distanza positiva nel verso di Y (positivo verso il fondo della macchina)
  f_base_distanza_tool_asse_divisore_mm_y = f_new_y_coordinate_mm - fPosizioneCentroRotazioneY_sign_OK_mm;
  % calcolo l'angolo naturale beta che il punto forma rispetto al piano XZ quando l'asse A è a zero gradi
  %// se faccio r*cos(beta) ottengo proprio la distanza asse divisore-punta utensile in Z
  %// e se faccio r*sin(beta) ottengo proprio la distanza asse divisore-punta utensile in Y
  f_beta_rad = atan2(f_base_distanza_tool_asse_divisore_mm_y, f_base_distanza_tool_asse_divisore_mm_z);
  %// distanza YZ fra posizione corrente della punta dell'utensile e l'asse del divisore
  %// e' praticamente l'analogo della sporgenza utensile nel caso della testa tf2
  %// con tf2 abbiamo sporgenza costante e posizione asse divisore variabile
  %// con la configurazione divisore 45 asse abbiamo sporgenza variabile e posizione asse divisore fissa
  f_base_distanza_tool_asse_divisore_mm_yz = sqrt( f_base_distanza_tool_asse_divisore_mm_y .^ 2 + f_base_distanza_tool_asse_divisore_mm_z .^ 2 );
  %// adesso devo però considerare che asse A si inclina di un angolo alfa=fPosRad[0]
  rotate_angle = f_beta_rad + sign_A * fPosRad_0;
  %// se inclino di un angolo alfa l'asse A, la nuova distanza in Z fra asse divisore e punta è data da r*(cos(beta+alfa))
  f_new_distanza_tool_asse_divisore_mm_z = f_base_distanza_tool_asse_divisore_mm_yz * cos(rotate_angle);
  %// se inclino di un angolo alfa l'asse A, la nuova distanza in Y fra asse divisore e punta è data da r*(sin(beta+alfa))
  f_new_distanza_tool_asse_divisore_mm_y = f_base_distanza_tool_asse_divisore_mm_yz * sin(rotate_angle);

  

%
%
% BEGIN TANGENTIAL AXIS HERE    
%
%
	if (rtcp_tangential_axis.do_calc)
    sign_angle_asse_tg = sign(rtcp_tangential_axis.f_alfa_asse_tg_no_offset_C_rot_rad);
		y_distance_from_y0_before_A_rotation_mm = f_base_distanza_tool_asse_divisore_mm_y;
    y_distance_from_y0_after_A_rotation_mm = f_new_distanza_tool_asse_divisore_mm_y;
    if (sign(y_distance_from_y0_before_A_rotation_mm) != sign(y_distance_from_y0_after_A_rotation_mm))
      sign_angle_asse_tg *= -1;
    endif
		rtcp_tangential_axis.f_alfa_asse_tg_no_offset_C_rot_A_rot_rad = sign_angle_asse_tg * atan2(abs(y_distance_from_y0_after_A_rotation_mm) * tan(rtcp_tangential_axis.f_alfa_asse_tg_no_offset_C_rot_rad), abs(y_distance_from_y0_before_A_rotation_mm));
	endif  
  
	f_alfa_asse_tg_delta_positive_clockwise_pulses = 0;
	if (rtcp_tangential_axis.do_calc)
  % calculates the position variation
		rtcp_tangential_axis.f_alfa_asse_tg_delta_positive_anticlockwise_rad = rtcp_tangential_axis.f_alfa_asse_tg_no_offset_C_rot_A_rot_rad - rtcp_tangential_axis.f_alfa_asse_tg_no_offset_rad;
		f_alfa_asse_tg_delta_positive_clockwise_pulses = -1 * rtcp_tangential_axis.f_alfa_asse_tg_delta_positive_anticlockwise_rad * (defRad2Deg) * dblDeg2Imp(defIndexAsseTg);
	endif
	rtcp_tangential_axis.f_output_alfa_asse_tg_delta_positive_clockwise_pulses = f_alfa_asse_tg_delta_positive_clockwise_pulses;
	rtcp_tangential_axis.valid = 1;  
%
%
% END TANGENTIAL AXIS HERE    
%
%
  
  
 % #ifdef def_rtcp_with_tangential_axis
  %add_calc_rtcp_tangential_axis_A_rotation(f_base_distanza_tool_asse_divisore_mm_y, f_new_distanza_tool_asse_divisore_mm_y);
  %close_calc_rtcp_tangential_axis();
  %#endif			

  %// calcola le nuove coordinate XYZ di incisione in base alla posizione corrente A e C e a quella XYZ
  %// intanto calcolo la variazione di posizione Z
  %// occhio che Z cresce verso il basso quindi la correzione va invertita
  l_delta_tcp_imp(defIndexAsseZ) = (-(f_new_distanza_tool_asse_divisore_mm_z - f_base_distanza_tool_asse_divisore_mm_z)) * dblMmImp(defIndexAsseZ);
  %// trovo la correzione in impulsi sommando anche eccentricity
  l_delta_tcp_imp(defIndexAsseX) = (f_new_x_coordinate_mm - f_act_x_coordinate_mm) * dblMmImp(defIndexAsseX);
  %// in Y devo stare attento al segno della correzione
  l_delta_tcp_imp(defIndexAsseY) = (f_new_distanza_tool_asse_divisore_mm_y + fPosizioneCentroRotazioneY_sign_OK_mm) * dblMmImp(defIndexAsseY) - l_pos_before_tcp_imp(defIndexAsseY);
  
  for i=defIndexAsseX:defIndexAsseZ
    xyzace_post_rtcp(i) = xyzace_pre_rtcp(i) + l_delta_tcp_imp(i);
  endfor;
  % adds tangential axis delta position
  xyzace_post_rtcp(defIndexAsseTg) = xyzace_pre_rtcp(defIndexAsseTg) + rtcp_tangential_axis.f_output_alfa_asse_tg_delta_positive_clockwise_pulses;

endfunction


