function varargout = LINK_INVALID_NAME(varargin)
  narginchk(0,1)
  if nargin==0
    nargoutchk(0,1)
    varargout{1} = iDynTreeMEX(802);
  else
    nargoutchk(0,0)
    iDynTreeMEX(803,varargin{1});
  end
end
