function hh = ShowPoly(p,varargin)

ms_assert('( size(p,1)==2 ) | ( size(p,1)==3 )')
ms_assert('size(p,2)>=3')

% if (size(p,1)>3)
%     warning('Transposing so that vertices are columns')
%     p=p.';   
% end

if (nargin==1)
    
    if (size(p,1)==2)
        out=patch(p(1,:)',p(2,:)','g');
    end
    
    if (size(p,1)==3)
        out=patch(p(1,:)',p(2,:)',p(3,:)','g');
    end
    
    if ( nargout > 0 )
        hh = out;
    end
    return
end


if (size(p,1)==2)
    out=patch(p(1,:)',p(2,:)','r',varargin{:});
end

if (size(p,1)==3)
    out = patch(p(1,:)',p(2,:)',p(3,:)','r',varargin{:});
    
    %out = patch( p(1,:)', p(2,:)', p(3,:)', varargin{:} );
    
    %Some problems are caused by using the wrong choice of the  above two lines
    %different choices work in different situations
end


if ( nargout > 0 )
    hh = out;
end

