function alfa = calcSteerAngle(delta)
wheelbase=2; %in metres
chordLength=6; %10*10 grid=1 m^2, so sqrt(2)*cellsize<chordLength is met
if delta>=2*atan(2*wheelbase/chordLength)
    error('Please correct chordLength or deltamax because they give unrealistic alfa values (calcSteerAngle.m)');
end

alfa=2*asin(chordLength/2/wheelbase*tan(delta/2));
end

