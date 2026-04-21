function [q_eu, q_ed] = ikinPickAndPlace(X,L)
    x = X(1);
    y = X(2);
    z = X(3);
    phi = X(4);

    l1 = L(1);
    l2 = L(2);
    l3 = L(3);
    l4 = L(4);
    l5 = L(5);
    l6 = L(6);
    
    a1 = -y;
    b1 = -x;
    c1 = sqrt((a1^2)+(b1^2));
    alpha1 = atan2(a1,b1);

    a2 = l4;
    b2 = sqrt((c1^2)-(a2^2));
    alpha2 = atan2(a2,b2);
    theta1 = alpha1 - alpha2;

    c3 = b2 -l5;

    a4 = (z + l6) - l1;
    c4 = sqrt((c3^2)+(a4^2));
    alpha4 = atan2(a4,c3);

    b5 = l2;
    a5 = l3;
    alpha5 = acos(((b5^2)+(c4^2)-(a5^2)) / (2*b5*c4));
    omega5 = acos(((a5^2)+(b5^2)-(c4^2)) / (2*a5*b5));

    theta2_up = -(alpha4+alpha5);
    theta3_up = pi -omega5;

    theta2_down = -(alpha4-alpha5);
    theta3_down = omega5-pi;

    theta4_up = -(theta2_up + theta3_up + (pi/2));
    theta4_down = -(theta2_down + theta3_down + (pi/2));

    theta6 = (pi/2) + theta1 - phi;

    theta5 = -(pi/2);

    q_eu = [theta1, theta2_up, theta3_up, theta4_up, theta5, theta6];
    q_ed = [theta1, theta2_down, theta3_down, theta4_down, theta5, theta6];
end