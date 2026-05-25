function jsonFieldsExist(data, fieldPaths)
    fieldPaths = string(fieldPaths);

    for fieldPath = fieldPaths
        if ~jsonPathExists(data, fieldPath)
            error("Error: key '%s' is not present in JSON", fieldPath);
        end
    end
end

function pathExists = jsonPathExists(data, fieldPath)
    parts = split(fieldPath, ".");
    current = data;

    pathExists = true;

    for part = parts'
        part = char(part);

        if ~isstruct(current) || ~isfield(current, part)
            pathExists = false;
            return;
        end

        current = current.(part);
    end
end