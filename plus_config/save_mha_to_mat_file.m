data = mha_read_transforms("data_acquisition.mha");

if sum(data.FlangeToBaseTransformStatus) == length(data.FlangeToBaseTransformStatus)  

    data_size = sum(data.ProbeToPolarisTransformStatus);

    T_BF = zeros(4,4,data_size);
    T_PT = zeros(4,4,data_size);

    k = 1;
    for i = 1:size(data.FlangeToBaseTransformMatrix,3)

        if data.ProbeToPolarisTransformStatus(i) == 1

            T_BF(:,:,k) = data.FlangeToBaseTransformMatrix(:,:,i);
            T_PT(:,:,k) = data.ProbeToPolarisTransformMatrix(:,:,i);

            k = k + 1;

        end
    end
    
    for i = 1:size(T_BF,3)
       T_BF(1:3,4,i) = T_BF(1:3,4,i)/1000; 
       T_PT(1:3,4,i) = T_PT(1:3,4,i)/1000;
    end

    save("acquired_data/robot_polaris_data_65.mat", "T_BF", "T_PT")

else
    error("Robot observations")
end