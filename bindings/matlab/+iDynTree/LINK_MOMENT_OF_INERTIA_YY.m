function v = LINK_MOMENT_OF_INERTIA_YY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 10);
  end
  v = vInitialized;
end