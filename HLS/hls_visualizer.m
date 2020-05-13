clear all;
traj = [
1.570796326734125614166259765625, 0
1.48278057784773409366607666015625, .11901440657675266265869140625
1.56625647121109068393707275390625, .24358660541474819183349609375
1.53800625703297555446624755859375, .3909100783057510852813720703125
1.60060242074541747570037841796875, .5273360093124210834503173828125
1.65865893266163766384124755859375, .66559947677887976169586181640625
1.775976455770432949066162109375, .75898856599815189838409423828125
1.79133694409392774105072021484375, .9082114393822848796844482421875
1.90199813642539083957672119140625, 1.00943195982836186885833740234375
1.9138332488946616649627685546875, 1.15900104679167270660400390625
1.9562881407327950000762939453125, 1.30286563001573085784912109375
1.88779604551382362842559814453125, 1.43642224068753421306610107421875
1.770267381332814693450927734375, 1.52963991300202906131744384765625
1.62132196803577244281768798828125, 1.54725745110772550106048583984375
1.54016350139863789081573486328125, 1.6734364661388099193572998046875
1.47824165155179798603057861328125, 1.81002463004551827907562255859375
1.35333390277810394763946533203125, 1.89314339798875153064727783203125
1.20500541874207556247711181640625, 1.915452948771417140960693359375
1.06444979668594896793365478515625, 1.86297272541560232639312744140625
.94097718480043113231658935546875, 1.7777192103676497936248779296875
.81215660111047327518463134765625, 1.70092152245342731475830078125
.67057879804633557796478271484375, 1.75068589183501899242401123046875
.5209172666072845458984375, 1.73919873661361634731292724609375
.3800807860679924488067626953125, 1.68768597138114273548126220703125
.25776865263469517230987548828125, 1.77459959662519395351409912109375
.11497823917306959629058837890625, 1.7285705171525478363037109375
-.03491769800893962383270263671875, 1.7316140118055045604705810546875
-.18315927684307098388671875, 1.70856197574175894260406494140625
-.18258819147013127803802490234375, 1.5585621302016079425811767578125
-.311304823495447635650634765625, 1.481557589955627918243408203125
-.3965251632034778594970703125, 1.35810664971359074115753173828125
-.5202086600475013256072998046875, 1.27328915358521044254302978515625
-.60847899713553488254547119140625, 1.15204091556370258331298828125
-.6346000232733786106109619140625, 1.00446956628002226352691650390625
-.559895536862313747406005859375, .87440447160042822360992431640625
-.4674847102724015712738037109375, .75623088586144149303436279296875
-.36318367696367204189300537109375, .648506878875195980072021484375
-.391179881989955902099609375, .50118659972213208675384521484375
-.27088208380155265331268310546875, .411602654494345188140869140625
-.14088919037021696567535400390625, .33682936779223382472991943359375
-.20089134271256625652313232421875, .19932215358130633831024169921875
-.065249786712229251861572265625, .13516027224250137805938720703125
0, 0

];

%[newP,newQ] = addConfigs(transpose(traj)) ;
%newP
%newP = transpose(traj);
x=traj(:,1)
y=traj(:,2)

L1 = 1;
L2 = 1;
for i=1:length(x)
    eeLoc(1,i) = L1*cos(x(i)) + L2*cos(x(i)+y(i));
    eeLoc(2,i) = L1*sin(x(i)) + L2*sin(x(i)+y(i)); 
end

eeLoc
figure(2); 
plot(eeLoc(1,:), eeLoc(2,:)); 
hold on;
r=0.275; %0.25;
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
plot(1.5+xp,1.5+yp);

figure(1);
L1 = 1;
L2 = 1;
for i=1:length(x)
    a(1) = 0;
    b(1) = 0;
    a(2) = L1*cos(x(i));
    b(2) = L1*sin(x(i));
    a(3) = L1*cos(x(i)) + L2*cos(x(i)+y(i));
    b(3) = L1*sin(x(i)) + L2*sin(x(i)+y(i));
    plot(a,b);
    hold on;
end
hold on;
r=0.275;
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
plot(1.5+xp,1.5+yp);

function q = inverseKinematics(p)
    % TODO: solve the inverse kinematics
    q = armInverseKinematics(p);
end

function [newP,newQ] = addConfigs(P) 
resolution = 25;
pointNum = 1;
for i=1:length(P)
    if i ~= length(P)
        newP(:,pointNum) = P(:,i);
        newQ(:,pointNum) = inverseKinematics(P(:,i));
        p1 = P(:,i);
        p2 = P(:,i+1);
        m = (p2(2) - p1(2))/(p2(1) - p1(1));
        xdiff = p2(1) - p1(1);
        pointNum = pointNum + 1; 
        pos(1,1) = p1(1);
        pos(2,1) = p1(2);
        for j=2:(resolution-1)
            p1(1) = pos(1,1) + j*(xdiff/resolution);
            p1(2) = pos(2,1) + m*(p1(1)-pos(1,1));
            newP(:,pointNum) = p1;
            newQ(:,pointNum) = inverseKinematics(p1);
            pointNum = pointNum + 1; 
            %try inverseKinematics(p1); 
            %    newP(:,pointNum) = p1;
            %    newQ(:,pointNum) = inverseKinematics(p1);
            %    pointNum = pointNum + 1; 
            %catch exception
            %    continue;
            %end
        end
    else
        newP(:,pointNum) = P(:,i);
        newQ(:,pointNum) = inverseKinematics(P(:,i));
    end
end
end