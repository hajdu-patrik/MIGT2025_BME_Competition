package rescueframework;

public enum Action {
	STEP_UP(0), 
	STEP_RIGHT(1), 
	STEP_DOWN(2), 
	STEP_LEFT(3), 
	PICK_UP(5), 
	PUT_DOWN(5),  
	HEAL(6),
	IDLE(null);
	
	private Integer value = null;
	
	Action(Integer value){
		this.value = value;
	}
	
	public Integer getValue() {
		return value;
	}
}
