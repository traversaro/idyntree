classdef Polygon2D < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = m_vertices(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1931, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1932, self, varargin{1});
      end
    end
    function self = Polygon2D(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1933, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = setNrOfVertices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1934, self, varargin{:});
    end
    function varargout = getNrOfVertices(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1935, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1936, self, varargin{:});
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1937, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1938, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
