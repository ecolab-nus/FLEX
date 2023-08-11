module ff_rst #(
    parameter NUM_STAGES = 2
    )(
    input  clk,
    input  rstn_master,
    input  rstn_in,
    output rstn_out
);
    //Variables
    int i;
    logic [NUM_STAGES-1:0] ff;

    //Reset synchronizer circuit
    always_ff @ (posedge clk or negedge rstn_master) begin
        if (!rstn_master) ff <= '0;
        else begin
            ff[0] <= rstn_in; //This should be 1'b1
            //Additional stages for delay chain
            for (i=1; i<NUM_STAGES; i=i+1) begin: ff_stage
            ff[i] <= ff[i-1];
            end
        end
    end

    //Synchronized reset output
    assign rstn_out = ff[NUM_STAGES-1];

endmodule


