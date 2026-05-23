function matVarsExist(data, varNames)
    existingNames = string(fieldnames(data));

    varNames = string(varNames);

    for varName = varNames
        if ~ismember(varName, existingNames)
            error("%s is not present in MAT data", varName);
        end
    end

end