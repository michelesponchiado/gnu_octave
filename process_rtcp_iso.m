function [coords_out, coords_out_iso] = process_rtcp_iso(iso_filename)
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
  while(1)
    s = fgets(f);
    if (s < 0)
      break;
    endif;
    [new_coordinates, error] = parse_iso_line(s, coords_prertcp);
    if (error)
      break;
    endif;
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
    coords_out(:, idx_coords_line) = coords_postrtcp(:);
    idx_coords_line = idx_coords_line + 1;
    %disp(coords_postrtcp);
  endwhile;
  fclose(f);
  plot3(coords_out(1,:),coords_out(2,:),coords_out(3,:));
  xlabel("X");
  ylabel("Y");
  zlabel("Z");
  % invert zdir
  set(gca,'zdir','reverse');
endfunction