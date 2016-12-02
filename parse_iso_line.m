function [new_coordinates, error] = parse_iso_line(line, current_coordinates)
  error = 0;
  new_coordinates = current_coordinates;
  for i = 1:length(line)
    c = line(i);
    c = toupper(c);
    if (c == '(')
      % comment out the line
      break;
    endif
    idx_dst = 1;
    set_pos = 0;
    if     c == 'X'
      idx_dst = 1;
      set_pos = 1;
    elseif c == 'Y'
      idx_dst = 2;
      set_pos = 1;
    elseif c == 'Z'
      idx_dst = 3;
      set_pos = 1;
    elseif c == 'A'
      idx_dst = 4;
      set_pos = 1;
    elseif c == 'C'
      idx_dst = 5;
      set_pos = 1;
    endif;
    if (set_pos > 0)
      pos_str = substr(line, i + 1);
      next_index = 0;
      val = 0;
      count = 0;
      ERRMSG = "";
      [val, count, ERRMSG, next_index] = sscanf (pos_str, "%f", length(pos_str));
      if (count == 1)
        % invert Z sign...
        if (idx_dst == 3)
          val = val * -1;
        endif
        new_coordinates(idx_dst) = val;
        i = next_index;
      else
        disp(ERRMSG);
        disp("Invalid input string at offset ");disp(i);disp(": ");disp(line);
        error = 1;
        break;
      endif;
    endif;
  endfor;
endfunction