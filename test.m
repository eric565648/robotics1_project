e=.025;eta=0.04;c=100;M=100;
hI=(-1:.05:3);
s=sigmafun(hI,eta,c,M,e);
figure(5);plot(hI,s,'-','linewidth',2);grid
xlabel('hI');ylabel('\sigma');title('control barrier function');

function s=sigmafun(hI,eta,c,M,e)

    s=(hI>eta).*(-M*atan(c*(hI-eta))*2/pi)+...
        (hI>=0).*(hI<eta).*(e*(eta-hI)/eta)+(hI<0).*e;

end