function v = LINK_FIRST_MOMENT_OF_MASS_Z()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 6);
  end
  v = vInitialized;
end