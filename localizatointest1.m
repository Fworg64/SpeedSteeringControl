iter = 1:50;
dt = .01;

yvector =( (.3 + .1)/2 * .01 .* iter) .* sin(.0660/.5 .* .01*iter) + normrnd(0,.02,1,size(iter,2));
xvector = ( (.3 + .1)/2 * .01 .* iter) .* cos(.0660/.5 .* .01*iter) + normrnd(0,.02,1,size(iter,2));
figure()
plot(yvector);
figure()
plot(xvector);