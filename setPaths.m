function [  ] = setPaths(  )
%
% expand the path to include the local directories
% Current file must include setPaths
%

   ROOT = cd;
   
   % modify the path to include the progam specific routines
   nPath = [ ROOT '/Main' ];     path( nPath, path ); % Main run functions
   nPath = [ ROOT '/Utility' ];  path( nPath, path ); % Utility functions

end