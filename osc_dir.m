function dir_vector = osc_dir(t)
    dir_vector = [-1 0 0]';
    dir_vector(2) = sin(t);
    dir_vector(1) = -1*(1-dir_vector(2)^2)^0.5;
end
    

