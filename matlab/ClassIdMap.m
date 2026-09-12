classdef ClassIdMap
    % CLASSIDMAP Map between class names and numeric IDs for Simulink signals
    %
    % Simulink ports carry fixed-type signals (double, int32, etc.). The pipeline
    % uses free-form class_name strings (e.g., "pedestrian", "animal", "vehicle"),
    % but these can't be passed through Simulink signal buses directly.
    %
    % This class centralizes the convention: a numeric ID ↔ string name mapping.
    % Used by:
    % - pipeline_wrapper.m (unpacking Python class_name into MATLAB class_id)
    % - Any YOLO/perception block providing detections upstream
    % - The Stateflow chart guards (e.g., "if nearest_class_id == CLASS_ANIMAL")
    %
    % OPEN INTERFACE ITEM (flagged in the plan):
    % The enum below assumes "pedestrian", "animal", "vehicle", "unknown".
    % This must be confirmed/aligned with whoever owns the perception/YOLO block
    % (not yet built in MATLAB; assumed to be a separate teammate's work).
    %
    % If the perception block produces different class_name values, add them here.

    properties (Constant)
        % Numeric IDs for each class (int32)
        CLASS_PEDESTRIAN = int32(0);
        CLASS_ANIMAL = int32(1);
        CLASS_VEHICLE = int32(2);
        CLASS_UNKNOWN = int32(3);

        % Maximum ID for validation
        MAX_CLASS_ID = int32(3);
    end

    methods (Static)
        function class_id = name_to_id(class_name)
            % NAME_TO_ID Convert a class name (string) to its numeric ID.
            %   class_id = ClassIdMap.name_to_id("pedestrian") -> 0
            %
            % Args:
            %   class_name: char array or string
            %
            % Returns:
            %   class_id: int32

            class_name = string(class_name);

            switch lower(class_name)
                case "pedestrian"
                    class_id = ClassIdMap.CLASS_PEDESTRIAN;
                case "animal"
                    class_id = ClassIdMap.CLASS_ANIMAL;
                case "vehicle"
                    class_id = ClassIdMap.CLASS_VEHICLE;
                otherwise
                    warning('ClassIdMap:UnknownClass', ...
                            'Class "%s" not in enum; mapping to UNKNOWN', ...
                            class_name);
                    class_id = ClassIdMap.CLASS_UNKNOWN;
            end
        end

        function class_name = id_to_name(class_id)
            % ID_TO_NAME Convert a numeric ID to its class name.
            %   class_name = ClassIdMap.id_to_name(0) -> "pedestrian"
            %
            % Args:
            %   class_id: int32 (or convertible to int)
            %
            % Returns:
            %   class_name: string

            class_id = int32(class_id);

            switch class_id
                case ClassIdMap.CLASS_PEDESTRIAN
                    class_name = "pedestrian";
                case ClassIdMap.CLASS_ANIMAL
                    class_name = "animal";
                case ClassIdMap.CLASS_VEHICLE
                    class_name = "vehicle";
                case ClassIdMap.CLASS_UNKNOWN
                    class_name = "unknown";
                otherwise
                    warning('ClassIdMap:InvalidId', ...
                            'ID %d outside enum range [0-%d]; returning "unknown"', ...
                            class_id, ClassIdMap.MAX_CLASS_ID);
                    class_name = "unknown";
            end
        end
    end
end
