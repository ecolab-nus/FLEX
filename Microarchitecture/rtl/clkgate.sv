
module clkgate #(
    //Clock gate switch off delay
    parameter CG_DELAY = 2
    )(
    //Clocking
    input          clk,
    input          clkn,
    input          clken,
    input          rstn,
    output logic   clk_g
);
    //Registers & wires
    int unsigned n;
    logic [CG_DELAY-1:0] clken_delay;

    //Clock enable shift circuit (use negative clock)
    always_ff @(posedge clkn or negedge rstn) begin
        if (!rstn) begin
            clken_delay <= {CG_DELAY{1'b1}}; //Must enable clock during reset phase to avoid reset glitches
        end

        else begin
            for (n=0; n<CG_DELAY; n++) begin
                if (n==0) clken_delay[n] <= clken;
                else if (n==CG_DELAY-1) clken_delay[n] <= clken || clken_delay[n-1];
                else clken_delay[n] <= clken_delay[n-1];
            end
        end
    end

    //Clock gate
    assign clk_g = clk && clken_delay[CG_DELAY-1];
endmodule

