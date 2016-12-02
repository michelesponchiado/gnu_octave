function [coords_out, coords_out_iso, trace_coords_1, trace_coords_2, xy_c, or_xy] = process_rtcp_iso(iso_filename)
  coords_out = zeros(8,1);
  coords_out_iso = zeros(8,1);
  coords_prertcp = zeros(8,1);
  coords_postrtcp = zeros(8,1);

  iso_origin = zeros(8,1);
  iso_origin(1) =  310.933990;
  iso_origin(2) = -257.444000;
  iso_origin(3) =  468.967438 - 125.0 - 50.0;
  
  defIndexAsseX = 1;
  defIndexAsseY = 2;
  defIndexAsseZ = 3;
  defIndexAsseA = 4;
  defIndexAsseC = 5;
  defIndexAsseTg = 7;
  
  dblMmImp = ones (8,1);
  dblMmImp(defIndexAsseX) = 499.975006;
  dblMmImp(defIndexAsseY) = 499.987488;
  dblMmImp(defIndexAsseZ) = 499.975006;
  dblImpMm = ones (8,1);
  dblImpMm(defIndexAsseX) = 1.0 / dblMmImp(defIndexAsseX);
  dblImpMm(defIndexAsseY) = 1.0 / dblMmImp(defIndexAsseY);
  dblImpMm(defIndexAsseZ) = 1.0 / dblMmImp(defIndexAsseZ);

  dblOffsetImpulsi = zeros (8,1);
  dblOffsetImpulsi(defIndexAsseA) = 144731.000000;
  dblOffsetImpulsi(defIndexAsseC) = 16737.000000;
  
  dblDeg2Imp = ones (8,1);
  dblDeg2Imp(defIndexAsseA) = 3000;
  dblDeg2Imp(defIndexAsseC) = 3000;
  dblDeg2Imp(defIndexAsseTg) = 1.422222;

  dblImp2Deg = ones (8,1);
  dblImp2Deg(defIndexAsseA) = 1.0 / dblDeg2Imp(defIndexAsseA);
  dblImp2Deg(defIndexAsseC) = 1.0 / dblDeg2Imp(defIndexAsseC);
  dblImp2Deg(defIndexAsseTg) = 1.0 / dblDeg2Imp(defIndexAsseTg);
  
  idx_coords_line = 1;

  [f, msg] = fopen(iso_filename, "rb");
  if (f < 0)
    display(msg);
    return;
  endif
  cnt_lines = 0;
  while(1)
    s = fgets(f);
    if (s < 0)
      break;
    endif;
    [new_coordinates, error] = parse_iso_line(s, coords_prertcp);
    if (error)
      break;
    endif;
    cnt_lines = cnt_lines + 1;
    %if (cnt_lines > 40)
    %  break;
    %endif;
    coords_from_iso_and_origin = new_coordinates(:) + iso_origin(:);
    coords_out_iso(:, idx_coords_line) = coords_from_iso_and_origin;
%    coords(:, idx_coords_line) = new_coordinates(:);
    %disp(new_coordinates);
    coords_prertcp = new_coordinates;
    coords_4_rtcp_imp = zeros(8, 1);
    for i = defIndexAsseX:defIndexAsseZ
      coords_4_rtcp_imp(i) = coords_from_iso_and_origin(i) .* dblMmImp(i);
    endfor;
    for i = defIndexAsseA:defIndexAsseC
      coords_4_rtcp_imp(i) = coords_from_iso_and_origin(i) .* dblDeg2Imp(i) + dblOffsetImpulsi(i);
    endfor;
    %disp(coords_prertcp);
    [coords_postrtcp_imp] = rtcp_apply(coords_4_rtcp_imp);
    for i = defIndexAsseX:defIndexAsseZ
      coords_postrtcp(i) = coords_postrtcp_imp(i) .* dblImpMm(i);
    endfor;
    for i = defIndexAsseA:defIndexAsseC
      coords_postrtcp(i) = (coords_postrtcp_imp(i) - dblOffsetImpulsi(i)) .* dblImp2Deg(i);
    endfor;
    coords_postrtcp(defIndexAsseTg) = (coords_postrtcp_imp(defIndexAsseTg) - dblOffsetImpulsi(defIndexAsseTg)) .* dblImp2Deg(defIndexAsseTg);
    coords_out(:, idx_coords_line) = coords_postrtcp(:);
    idx_coords_line = idx_coords_line + 1;
    %disp(coords_postrtcp);
  endwhile;
  fclose(f);
  if (1)
    xy_c = zeros(3, length(coords_out));
    or_xy = zeros(2, length(coords_out));
    trace_coords_1 = zeros(3, length(coords_out));
    trace_coords_2 = zeros(3, length(coords_out));
    dxyz = zeros(3,3);
    orig = zeros(3,1);
    r = 50;
    orig_rotate_A = [iso_origin(1), iso_origin(2), 468.967438 - 125.0];
    orig_rotate_C = [iso_origin(1), iso_origin(2), 468.967438 - 125.0 - r];
    trace_coords_1(1:3, 1) = coords_out(1:3 , 1);
    trace_coords_2(1:3, 1) = coords_out(1:3 , 1);
    x_orig = orig_rotate_A(1);
    for i=2:length(coords_out)
      orig = [orig_rotate_A(1), orig_rotate_A(2) - r * sin(coords_out(4,i)*pi/180), orig_rotate_A(3) + r * (1 - cos(coords_out(4,i)*pi/180))];
      dxyz(3, :) = dxyz(2, :);
      dxyz(2, :) = dxyz(1, :);
      dxyz(1, :) = [coords_out(1,i) - x_orig, coords_out(2,i) - orig(2), coords_out(3,i) - orig(3)];
      trace_coords_1(:, i) = orig + dxyz(2, :);
      trace_coords_2(:, i) = orig + dxyz(3, :);
      alfa = -1*coords_out(5,i) * pi/180;
      xy_c(1 , i) = dxyz(1, 1) * cos(alfa) - dxyz(1, 2) * sin(alfa) ;
      xy_c(2 , i) = dxyz(1, 1) * sin(alfa) + dxyz(1, 2) * cos(alfa) ;
      xy_c(3 , i) = coords_out(5,i);
      or_xy(1, i) = orig(1);
      or_xy(2, i) = orig(2);
    endfor 
    
  else
    plot3(coords_out(1,:),coords_out(2,:),coords_out(3,:));
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
    % invert zdir
    set(gca,'zdir','reverse');
  endif;
endfunction