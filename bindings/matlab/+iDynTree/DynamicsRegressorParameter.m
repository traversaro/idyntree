classdef DynamicsRegressorParameter < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = category(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1013, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1014, self, varargin{1});
      end
    end
    function varargout = elemIndex(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1015, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1016, self, varargin{1});
      end
    end
    function varargout = type(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1017, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1018, self, varargin{1});
      end
    end
    function varargout = lt(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1019, self, varargin{:});
    end
    function varargout = eq(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1020, self, varargin{:});
    end
    function varargout = ne(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1021, self, varargin{:});
    end
    function self = DynamicsRegressorParameter(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if varargin{1}~=SwigRef.Null
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1022, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1023, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
