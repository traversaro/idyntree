classdef MomentumFreeFloatingJacobian < iDynTree.MatrixDynSize
  methods
    function self = MomentumFreeFloatingJacobian(varargin)
      self@iDynTree.MatrixDynSize(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1138, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1139, self, varargin{:});
    end
    function varargout = isConsistent(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1140, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1141, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
