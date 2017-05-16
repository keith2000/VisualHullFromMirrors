function q = convexhull(p)

%reurn the subset of points that lie on the convex hull for 2 or 3 dimensions



if ( isfield(p, 'boundary') )
    
    q = p;
    for viewLoop = 1:length(p),    
        q(viewLoop).boundary = convexhull(p(viewLoop).boundary);
    end
    return
    
else
    ensure('any( size(p,1) == [2,3])', 'Input points must be 2xn or 3xn' )     
    ensure('size(p,2) > size(p,1)')
    if size(p,1) == 2
        k = convhull( p(1,:), p(2,:) );
        q = p(:, k(1:end-1) );        
        
        return
    end
    
    if size(p,1) == 3
        k = convhulln( p' );
        q = p(:, unique(k) );        
        return
    end
end

error('Should never be able to get here!')

% 
% %q=convexhull(p)
% %returns the convex hull rather than the vertex indices
% 
% if (size(p,1)==2)
%     
%     
%     if (size(p,2)<=3)
%         error('Polygons must consist of at least 3 vertices')
%     end
%     
%     
%     lastLength = 0;
%     
%     while ( lastLength~=size(p,2) )
%         
%         lastLength = size(p,2);
%         done = 0;
%         while ( done == 0 )
%             kk = convhull( p(1,:), p(2,:) );
%             done =1;
%         end
%         p = p(:, kk(1:end-1) );
%     end
%     q=p;
%     
%     
%     if 0
%         kk = convhull( p(1,:), p(2,:) );
%         q = p(:, kk(1:end-1) );
%     end
%     
%     
%     %convhull seems to work better than the below:
%     % 
%     % if 0
%     %     
%     %     lastLength = 0;
%     %     
%     %     while (lastLength~=size(p,2))
%     %         
%     %         lastLength=size(p,2);
%     %         %  k=convhull(p(1,:),p(2,:));
%     %         
%     %         done = 0;
%     %         while done==0
%     %             try
%     %                 k=convex2(p(1,:),p(2,:));
%     %                 done =1;
%     %             catch
%     %                 p=p.*(1+0.001*randn(size(p)));
%     %                 disp(lasterr)
%     %                 disp('small random adjustment being made as a kludge...')
%     %             end
%     %         end
%     %         
%     %         p=p(:,k);
%     %         
%     %         %    disp(k)
%     %     end
%     %     
%     %     q=p;
%     % end
% else
%     
%     if (size(p,1)==3)
%         
%         k = convhulln( p' );
%         q = p(:, unique(k) );
%         
%     else
%         error('Input points must be 2xn or 3xn')    
%     end
%     
%     
%     
% end