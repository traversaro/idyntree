function varargout = LINK_INVALID_NAME(varargin)
  narginchk(0,1)
  if nargin==0
    nargoutchk(0,1)
    varargout{1} = iDynTreeMATLAB_wrap(334);
  else
    nargoutchk(0,0)
    iDynTreeMATLAB_wrap(335,varargin{1});
  end
end