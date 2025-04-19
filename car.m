classdef car < matlab.mixin.Copyable
    properties
        Front (6,1) = [2 0 0 0 0 0]
        Rear (6,1) = [0 0 0 0 0 0]
        Centre (6,1) = [1 0 0 0 0 0]
        Speed  = 1 
        SteeringAngle = [0 0]
        Length = 2
        h = 0.005
        dt= 0
        f 
        M
        j=1;
    end
    methods
        function obj=car(x0,th0,v,steeringAngle,h,L,M)
            %-pi<thstart<pi
            if nargin >0
                obj.Speed=v;
                obj.SteeringAngle=steeringAngle;
                obj.h=h;
                obj.Front(3)=steeringAngle(1);
                obj.Length=L;
                obj.Front(1:3) = [x0(1)+L*cos(th0) x0(2)+L*sin(th0) th0+obj.SteeringAngle(1)];
                obj.Centre(1:3) = [x0(1)+L/2*cos(th0) x0(2)+L/2*sin(th0) th0];
                if obj.Centre(3)>pi
                    obj.Centre(3)=obj.Centre(3)-2*pi;
                elseif obj.Centre(3)<-pi
                    obj.Centre(3)=obj.Centre(3)+2*pi;
                end
                
                obj.Rear(1:3) = [x0(1) x0(2) obj.Centre(3)];
                
                if size(M,2)~=2
                    M=M';
                end
                obj.M=M;

                obj.f=costFun(obj);
                
            end
        end
        function obj=updateState(obj)
            beta=atan(tan(obj.SteeringAngle(1))/2);
            temp=obj.SteeringAngle(1)+obj.h*obj.SteeringAngle(2);
            if abs(temp)<2/8*pi
                obj.SteeringAngle(1)=temp;
            else
                obj.SteeringAngle(1)=sign(temp)*2/8*pi;
            end
            obj.Centre(6)=obj.Speed*tan(obj.SteeringAngle(1))*cos(beta)/obj.Length;
            obj.Centre(3)=mod(obj.Centre(3)+obj.h*obj.Centre(6)+pi,2*pi)-pi;

            
            obj.Centre(4:5)=obj.Speed*[cos(obj.Centre(3)+beta) sin(obj.Centre(3)+beta)]';
            obj.Centre(1:2)=obj.Centre(1:2)+obj.h*obj.Centre(4:5);
            
            obj.Front(1:2)=obj.Centre(1:2)+[obj.Length/2*cos(obj.Centre(3)) obj.Length/2*sin(obj.Centre(3)) ]';
            obj.Front(3)=mod(obj.Centre(3)+obj.SteeringAngle(1)+pi,2*pi)-pi;

            obj.Rear(1:3)=obj.Centre(1:3)-obj.Length/2*[cos(obj.Centre(3)) sin(obj.Centre(3)) 0]';
            obj.dt=obj.dt+obj.h;
            obj.f=costFun(obj);
        end
        
        function obj=steer(obj,ddelta)
            obj.SteeringAngle(2)=sign(ddelta)*min(pi/4,abs(ddelta));
        end
        function g=costFun(obj)
            [d,o]=obj.dist(obj.M);

            g=log(d^2)+tan(abs(o)/2)+tan(obj.SteeringAngle(1)*2)^2;

        end
        function [d,o,j]=dist(obj,M)
            C=obj.Centre;

            rel=M-C(1:2)';
            vis=100;
            [q, param]=min(sum(rel.^2,2));
            G=rel(param,:);
            
            op=(G(:,2)*sin(C(3))-G(:,1)*cos(C(3)));
            [i, j]=min(abs(op));
            op2=op;
            op2(j)=[Inf];
            [i2, j2]=min(abs(op2));
            if abs(i-i2)<0.001
                if norm(G(j2,:))<norm(G(j,:))
                    j=j2;
                end
            end
            if norm(G(j,:))<obj.h*obj.Speed
                obj.j=j;
            end
            d=norm(G(j,:));
            
            phi=atan2(G(j,2),G(j,1));
%             if (j>1 && j<size(M,1))
%                 phi= atan2(M(j+1,2) - M(j-1,2),M(j+1,1) - M(j-1,1));
%             elseif j<size(M,1)
%                 phi= atan2(M(j+1,2) - M(j,2),M(j+1,1) - M(j,1));
%             elseif j>1
%                 phi= atan2(M(j,2) - M(j-1,2),M(j,1) - M(j-1,1));
%             else
%                 throw(MException("MyComponent:noSuchVariable","Not enough points in F"));
%             end

            o=C(3)-phi;
            if o>=pi
                o=2*pi-o;
            end
            if o<=-pi
                o=-2*pi-o;
            end
        end
    end
end